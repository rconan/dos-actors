use crate::{
    windloads::{WindLoads, WindLoadsBuilder},
    CfdLoads, Result, CS, M1S, M2S, MAX_DURATION,
};
use geotrans::{SegmentTrait, Transform};
use parse_monitors::{Exertion, Monitors, Vector};
use serde::{Deserialize, Serialize};
use std::mem;

/// [CfdLoads] builder
#[derive(Default, Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Builder<S> {
    pub(crate) cfd_case: String,
    pub(crate) duration: Option<f64>,
    pub(crate) time_range: Option<(f64, f64)>,
    pub(crate) nodes: Option<Vec<(String, CS)>>,
    pub(crate) upsampling: S,
    pub(crate) windloads: WindLoadsBuilder,
}
impl<S: Default> Builder<S> {
    /// Sets the wind loads time duration
    ///
    /// The duration is counted from the end of the recording
    pub fn duration(self, duration: f64) -> Self {
        Self {
            duration: Some(duration),
            ..self
        }
    }
    /// Sets the wind loads time range
    pub fn time_range(self, range: (f64, f64)) -> Self {
        Self {
            time_range: Some(range),
            ..self
        }
    }
    /// Sets the nodes `[x,y,z]` coordinates where the loads are applied
    pub fn nodes(self, keys: Vec<String>, locations: Vec<CS>) -> Self {
        assert!(
            keys.len() == locations.len(),
            "the number of wind loads node locations ({}) do not match the number of keys ({})",
            locations.len(),
            keys.len()
        );
        let nodes: Vec<_> = keys
            .into_iter()
            .zip(locations.into_iter())
            .map(|(x, y)| (x, y))
            .collect();
        Self {
            nodes: Some(nodes),
            ..self
        }
    }
    /// Requests mount segments loads
    pub fn mount(mut self, loads: Option<Vec<WindLoads>>) -> Self {
        self.windloads = self.windloads.mount(loads);
        self
    }
    /// Requests M1 segments loads
    pub fn m1_segments(mut self) -> Self {
        self.windloads = self.windloads.m1_segments();
        self
    }
    /// Requests M2 segments loads
    pub fn m2_segments(mut self) -> Self {
        self.windloads = self.windloads.m2_assembly();
        self
    }
    /// Selects the wind loads and filters the FEM
    #[cfg(fem)]
    fn loads(&mut self, loads: Vec<WindLoads>, fem: &mut gmt_fem::FEM) -> &mut Self {
        #[cfg(cfd2025)]
        let loads_index = <gmt_fem::FEM as gmt_dos_clients_fem::Model>::in_position::<
            gmt_dos_clients_io::gmt_fem::inputs::CFD2025046F,
        >(fem)
        .expect("missing input CFD2025046F in GMT FEM");
        #[cfg(cfd2021)]
        let loads_index = <gmt_fem::FEM as gmt_dos_clients_fem::Model>::in_position::<
            gmt_dos_clients_io::gmt_fem::inputs::CFD2021106F,
        >(fem)
        .expect("missing input CFD2021106F in GMT FEM");
        // filter FEM CFD input based on the selected CFD wind loads outputs
        fem.remove_inputs_by(&[loads_index], |x| {
            loads
                .iter()
                .flat_map(|x| x.fem())
                .fold(false, |b, p| b || x.descriptions.contains(&p))
        });
        // collect the descriptions of the FEM CFD filtered input
        let descriptions: Vec<_> = fem.inputs[loads_index]
            .as_ref()
            .map(|i| i.get_by(|x| Some(x.descriptions.clone())))
            .unwrap()
            .into_iter()
            .step_by(6)
            .collect();
        // CFD loads according to the FEM CFD descriptions
        let mut loads: Vec<_> = descriptions
            .iter()
            .map(|d| {
                loads
                    .iter()
                    .find_map(|l| l.fem().iter().find(|f| d.contains(*f)).and(Some(l)))
                    .unwrap()
            })
            .collect();
        loads.dedup();
        let keys: Vec<_> = loads.iter().flat_map(|l| l.keys()).collect();
        let info = descriptions
            .into_iter()
            .zip(&keys)
            .enumerate()
            .map(|(j, (x, k))| format!("{:2}. {} <-> {}", j + 1, k, x))
            .collect::<Vec<String>>()
            .join("\n");
        log::info!("\n{:}", info);
        let locations: Vec<CS> = fem.inputs[loads_index]
            .as_ref()
            .unwrap()
            .get_by(|x| Some(CS::OSS(x.properties.location.as_ref().unwrap().clone())))
            .into_iter()
            .step_by(6)
            .collect();
        assert!(
            keys.len() == locations.len(),
            "the number of wind loads node locations ({}) on input #{} do not match the number of keys ({})",
            locations.len(),
            loads_index,
            keys.len()
        );
        let nodes: Vec<_> = keys
            .into_iter()
            .zip(locations.into_iter())
            .map(|(x, y)| (x, y))
            .collect();
        match &mut self.nodes {
            Some(n) => n.extend(nodes),
            None => self.nodes = Some(nodes),
        };
        self
    }
    /// Filters the CFD inputs of the FEM according to the wind loads builder
    #[cfg(fem)]
    pub fn windloads(mut self, fem: &mut gmt_fem::FEM, mut builder: WindLoadsBuilder) -> Self {
        self.loads(builder.windloads, fem);
        if let Some(nodes) = builder.m1_nodes.take() {
            self.nodes.as_mut().map(|n| n.extend(nodes));
        }
        if let Some(nodes) = builder.m2_nodes.take() {
            self.nodes.as_mut().map(|n| n.extend(nodes));
        }
        self
    }
}

#[cfg(any(
    cfd2021,
    all(feature = "cfd2021", not(cfd2025), not(feature = "cfd2025"))
))]
impl<S> Builder<S> {
    /// Returns a [CfdLoads] object
    pub fn build(self) -> Result<CfdLoads<S>> {
        //println!("Loading the CFD loads from {} ...", self.cfd_case);
        //let now = Instant::now();
        let mut monitors = if let Some(time_range) = self.time_range {
            Monitors::loader::<String, 2021>(self.cfd_case)
                .start_time(time_range.0)
                .end_time(time_range.1)
                .load()?
        } else {
            Monitors::loader::<String, 2021>(self.cfd_case).load()?
        };

        let fm = monitors.forces_and_moments.remove("Cabs").unwrap();
        monitors
            .forces_and_moments
            .get_mut("platform")
            .unwrap()
            .iter_mut()
            .zip(fm.into_iter())
            .for_each(|(p, c)| {
                let u = p.clone();
                *p = &u + &c;
            });
        let fm = monitors.forces_and_moments.remove("cabletrays").unwrap();
        monitors
            .forces_and_moments
            .get_mut("platform")
            .unwrap()
            .iter_mut()
            .zip(fm.into_iter())
            .for_each(|(p, c)| {
                let u = p.clone();
                *p = &u + &c;
            });

        //println!(" - data loaded in {}s", now.elapsed().as_secs());
        /*if let Some(duration) = self.duration {
            let d = duration.ceil() as usize;
            monitors.keep_last(MAX_DURATION.min(d)); //.into_local();
        }*/
        let n_sample = match self.duration {
            Some(duration) => {
                let d = duration.ceil() as usize;
                monitors.keep_last(MAX_DURATION.min(d)); //.into_local();
                d * 20 + 1
            }
            None => monitors.len(),
        };
        let mut fm: Option<Vec<Option<Vec<f64>>>> = None;
        let mut m1_fm: Option<Vec<Option<Vec<f64>>>> = None;
        let mut m2_fm: Option<Vec<Option<Vec<f64>>>> = None;
        let mut total_exertion: Vec<Exertion> = vec![];
        if let Some(ref nodes) = self.nodes {
            for i in 0..monitors.len() {
                total_exertion.push(Exertion {
                    force: Vector::zero(),
                    moment: Vector::zero(),
                    cop: None,
                });
                for (key, location) in nodes.iter() {
                    let mut m1_cell = monitors
                        .forces_and_moments
                        .get_mut("M1cell")
                        .expect("M1cell not found in CFD loads")
                        .clone();
                    let exertion = monitors
                        .forces_and_moments
                        .get_mut(key)
                        .expect(&format!("{key} not found in CFD loads"));
                    let u = total_exertion[i].clone();
                    total_exertion[i] = &u + &exertion[i].clone();
                    match location {
                        CS::OSS(loc) => {
                            exertion[i].into_local(loc.into());
                            if let Some(fm) = fm.as_mut() {
                                fm.push((&exertion[i]).into());
                            } else {
                                fm = Some(vec![(&exertion[i]).into()]);
                            }
                        }
                        CS::M1S(j) => {
                            if *j < 2 {
                                let u = total_exertion[i].clone();
                                total_exertion[i] = &u + &m1_cell[i].clone();
                            }
                            let t: [f64; 3] = M1S::new(*j)?.translation().into();
                            exertion[i].into_local(t.into());
                            //if *j < 7 {
                            m1_cell[i].into_local(t.into());
                            if let Some(m1_cell) = &m1_cell[i] / 7f64 {
                                let v = &exertion[i] + &m1_cell;
                                exertion[i] = v;
                            }
                            //}
                            if let (Some(f), Some(m)) = (
                                Into::<Option<[f64; 3]>>::into(&exertion[i].force),
                                Into::<Option<[f64; 3]>>::into(&exertion[i].moment),
                            ) {
                                exertion[i].force = f.vfrov(M1S::new(*j))?.into();
                                exertion[i].moment = m.vfrov(M1S::new(*j))?.into();
                            };
                            if let Some(m1_fm) = m1_fm.as_mut() {
                                m1_fm.push((&exertion[i]).into());
                            } else {
                                m1_fm = Some(vec![(&exertion[i]).into()]);
                            }
                        }
                        CS::M2S(j) => {
                            let t: [f64; 3] = M2S::new(*j)?.translation().into();
                            exertion[i].into_local(t.into());
                            if let (Some(f), Some(m)) = (
                                Into::<Option<[f64; 3]>>::into(&exertion[i].force),
                                Into::<Option<[f64; 3]>>::into(&exertion[i].moment),
                            ) {
                                exertion[i].force = f.vfrov(M2S::new(*j))?.into();
                                exertion[i].moment = m.vfrov(M2S::new(*j))?.into();
                            };
                            if let Some(m2_fm) = m2_fm.as_mut() {
                                m2_fm.push((&exertion[i]).into());
                            } else {
                                m2_fm = Some(vec![(&exertion[i]).into()]);
                            }
                        }
                    };
                }
            }
        } else {
            for i in 0..monitors.len() {
                for exertion in monitors.forces_and_moments.values() {
                    if let Some(fm) = fm.as_mut() {
                        fm.push((&exertion[i]).into());
                    } else {
                        fm = Some(vec![(&exertion[i]).into()]);
                    }
                }
            }
        }

        let n = total_exertion.len() as f64;
        let force_mean = (total_exertion
            .iter()
            .fold(Vector::zero(), |a, e| a + e.force.clone())
            / n)
            .unwrap();
        let mut force_std = total_exertion
            .iter()
            .map(|e| (e.force.clone() - force_mean.clone()).unwrap())
            .map(|v| {
                let a: Option<Vec<f64>> = v.into();
                let a = a.unwrap();
                vec![a[0] * a[0], a[1] * a[1], a[2] * a[2]]
            })
            .fold(vec![0f64; 3], |mut a, e| {
                a.iter_mut().zip(e.iter()).for_each(|(a, e)| {
                    *a += e;
                });
                a
            });
        force_std.iter_mut().for_each(|x| *x = (*x / n).sqrt());
        log::info!(
            " OSS force: mean = {:.0?}N ; std = {:.0?}N",
            force_mean,
            force_std
        );
        let moment_mean = (total_exertion
            .iter()
            .fold(Vector::zero(), |a, e| a + e.moment.clone())
            / n)
            .unwrap();
        let mut moment_std = total_exertion
            .iter()
            .map(|e| (e.moment.clone() - moment_mean.clone()).unwrap())
            .map(|v| {
                let a: Option<Vec<f64>> = v.into();
                let a = a.unwrap();
                vec![a[0] * a[0], a[1] * a[1], a[2] * a[2]]
            })
            .fold(vec![0f64; 3], |mut a, e| {
                a.iter_mut().zip(e.iter()).for_each(|(a, e)| {
                    *a += e;
                });
                a
            });
        moment_std.iter_mut().for_each(|x| *x = (*x / n).sqrt());
        log::info!(
            " OSS moment: mean = {:.0?}N.m ; std = {:.0?}N.m",
            moment_mean,
            moment_std
        );

        let mut data: Option<Vec<f64>> = if let Some(fm) = fm {
            Some(fm.into_iter().filter_map(|x| x).flatten().collect())
        } else {
            None
        };
        let mut m1_loads: Option<Vec<f64>> = if let Some(fm) = m1_fm {
            Some(fm.into_iter().filter_map(|x| x).flatten().collect())
        } else {
            None
        };
        let mut m2_loads: Option<Vec<f64>> = if let Some(fm) = m2_fm {
            Some(fm.into_iter().filter_map(|x| x).flatten().collect())
        } else {
            None
        };
        let n = data
            .as_ref()
            .map_or(m1_loads.as_ref().map_or(0, |x| x.len()), |x| x.len())
            / monitors.time.len();
        if n_sample > monitors.len() {
            if let Some(ref mut data) = data {
                let mut v = data.clone();
                while n_sample * n > v.len() {
                    v = v
                        .chunks(n)
                        .chain(v.chunks(n).rev().skip(1))
                        .take(n_sample)
                        .flat_map(|x| x.to_vec())
                        .collect();
                }
                mem::swap(data, &mut v);
            }
            if let Some(ref mut data) = m1_loads {
                let mut v = data.clone();
                let n = 42;
                while n_sample * n > v.len() {
                    v = v
                        .chunks(n)
                        .chain(v.chunks(n).rev().skip(1))
                        .take(n_sample)
                        .flat_map(|x| x.to_vec())
                        .collect();
                }
                mem::swap(data, &mut v);
            }
            if let Some(ref mut data) = m2_loads {
                let mut v = data.clone();
                let n = 42;
                while n_sample * n > v.len() {
                    v = v
                        .chunks(n)
                        .chain(v.chunks(n).rev().skip(1))
                        .take(n_sample)
                        .flat_map(|x| x.to_vec())
                        .collect();
                }
                mem::swap(data, &mut v);
            }
        }
        Ok(CfdLoads {
            oss: data,
            m1: m1_loads,
            m2: m2_loads,
            nodes: self.nodes,
            n_fm: n,
            step: 0,
            upsampling: self.upsampling,
            max_step: usize::MAX,
        })
    }
}

#[cfg(any(
    cfd2025,
    all(feature = "cfd2025", not(cfd2021), not(feature = "cfd2021"))
))]
impl<S> Builder<S> {
    /// Returns a [CfdLoads] object
    pub fn build(self) -> Result<CfdLoads<S>> {
        //println!("Loading the CFD loads from {} ...", self.cfd_case);
        //let now = Instant::now();
        let mut monitors = if let Some(time_range) = self.time_range {
            Monitors::loader::<String, 2025>(self.cfd_case)
                .start_time(time_range.0)
                .end_time(time_range.1)
                .load()?
        } else {
            Monitors::loader::<String, 2025>(self.cfd_case).load()?
        };

        // let fm = monitors.forces_and_moments.remove("Cabs").unwrap();
        // monitors
        //     .forces_and_moments
        //     .get_mut("platform")
        //     .unwrap()
        //     .iter_mut()
        //     .zip(fm.into_iter())
        //     .for_each(|(p, c)| {
        //         let u = p.clone();
        //         *p = &u + &c;
        //     });
        // let fm = monitors.forces_and_moments.remove("cabletrays").unwrap();
        // monitors
        //     .forces_and_moments
        //     .get_mut("platform")
        //     .unwrap()
        //     .iter_mut()
        //     .zip(fm.into_iter())
        //     .for_each(|(p, c)| {
        //         let u = p.clone();
        //         *p = &u + &c;
        //     });

        //println!(" - data loaded in {}s", now.elapsed().as_secs());
        /*if let Some(duration) = self.duration {
            let d = duration.ceil() as usize;
            monitors.keep_last(MAX_DURATION.min(d)); //.into_local();
        }*/
        let n_sample = match self.duration {
            Some(duration) => {
                let d = duration.ceil() as usize;
                monitors.keep_last(MAX_DURATION.min(d)); //.into_local();
                d * 20 + 1
            }
            None => monitors.len(),
        };
        let mut fm: Option<Vec<Option<Vec<f64>>>> = None;
        let mut m1_fm: Option<Vec<Option<Vec<f64>>>> = None;
        let mut m2_fm: Option<Vec<Option<Vec<f64>>>> = None;
        let mut total_exertion: Vec<Exertion> = vec![];
        if let Some(ref nodes) = self.nodes {
            for i in 0..monitors.len() {
                total_exertion.push(Exertion {
                    force: Vector::zero(),
                    moment: Vector::zero(),
                    cop: None,
                });
                for (key, location) in nodes.iter() {
                    // let mut m1_cell = monitors
                    //     .forces_and_moments
                    //     .get_mut("M1cell")
                    //     .expect("M1cell not found in CFD loads")
                    //     .clone();
                    let exertion = monitors
                        .forces_and_moments
                        .get_mut(key)
                        .expect(&format!("{key} not found in CFD loads"));
                    let u = total_exertion[i].clone();
                    total_exertion[i] = &u + &exertion[i].clone();
                    match location {
                        CS::OSS(loc) => {
                            exertion[i].into_local(loc.into());
                            if let Some(fm) = fm.as_mut() {
                                fm.push((&exertion[i]).into());
                            } else {
                                fm = Some(vec![(&exertion[i]).into()]);
                            }
                        }
                        CS::M1S(j) => {
                            // if *j < 2 {
                            //     let u = total_exertion[i].clone();
                            //     total_exertion[i] = &u + &m1_cell[i].clone();
                            // }
                            let t: [f64; 3] = M1S::new(*j)?.translation().into();
                            exertion[i].into_local(t.into());
                            //if *j < 7 {
                            // m1_cell[i].into_local(t.into());
                            // if let Some(m1_cell) = &m1_cell[i] / 7f64 {
                            //     let v = &exertion[i] + &m1_cell;
                            //     exertion[i] = v;
                            // }
                            //}
                            if let (Some(f), Some(m)) = (
                                Into::<Option<[f64; 3]>>::into(&exertion[i].force),
                                Into::<Option<[f64; 3]>>::into(&exertion[i].moment),
                            ) {
                                exertion[i].force = f.vfrov(M1S::new(*j))?.into();
                                exertion[i].moment = m.vfrov(M1S::new(*j))?.into();
                            };
                            if let Some(m1_fm) = m1_fm.as_mut() {
                                m1_fm.push((&exertion[i]).into());
                            } else {
                                m1_fm = Some(vec![(&exertion[i]).into()]);
                            }
                        }
                        CS::M2S(j) => {
                            let t: [f64; 3] = M2S::new(*j)?.translation().into();
                            exertion[i].into_local(t.into());
                            if let (Some(f), Some(m)) = (
                                Into::<Option<[f64; 3]>>::into(&exertion[i].force),
                                Into::<Option<[f64; 3]>>::into(&exertion[i].moment),
                            ) {
                                exertion[i].force = f.vfrov(M2S::new(*j))?.into();
                                exertion[i].moment = m.vfrov(M2S::new(*j))?.into();
                            };
                            if let Some(m2_fm) = m2_fm.as_mut() {
                                m2_fm.push((&exertion[i]).into());
                            } else {
                                m2_fm = Some(vec![(&exertion[i]).into()]);
                            }
                        }
                    };
                }
            }
        } else {
            for i in 0..monitors.len() {
                for exertion in monitors.forces_and_moments.values() {
                    if let Some(fm) = fm.as_mut() {
                        fm.push((&exertion[i]).into());
                    } else {
                        fm = Some(vec![(&exertion[i]).into()]);
                    }
                }
            }
        }

        let n = total_exertion.len() as f64;
        let force_mean = (total_exertion
            .iter()
            .fold(Vector::zero(), |a, e| a + e.force.clone())
            / n)
            .unwrap();
        let mut force_std = total_exertion
            .iter()
            .map(|e| (e.force.clone() - force_mean.clone()).unwrap())
            .map(|v| {
                let a: Option<Vec<f64>> = v.into();
                let a = a.unwrap();
                vec![a[0] * a[0], a[1] * a[1], a[2] * a[2]]
            })
            .fold(vec![0f64; 3], |mut a, e| {
                a.iter_mut().zip(e.iter()).for_each(|(a, e)| {
                    *a += e;
                });
                a
            });
        force_std.iter_mut().for_each(|x| *x = (*x / n).sqrt());
        log::info!(
            " OSS force: mean = {:.0?}N ; std = {:.0?}N",
            force_mean,
            force_std
        );
        let moment_mean = (total_exertion
            .iter()
            .fold(Vector::zero(), |a, e| a + e.moment.clone())
            / n)
            .unwrap();
        let mut moment_std = total_exertion
            .iter()
            .map(|e| (e.moment.clone() - moment_mean.clone()).unwrap())
            .map(|v| {
                let a: Option<Vec<f64>> = v.into();
                let a = a.unwrap();
                vec![a[0] * a[0], a[1] * a[1], a[2] * a[2]]
            })
            .fold(vec![0f64; 3], |mut a, e| {
                a.iter_mut().zip(e.iter()).for_each(|(a, e)| {
                    *a += e;
                });
                a
            });
        moment_std.iter_mut().for_each(|x| *x = (*x / n).sqrt());
        log::info!(
            " OSS moment: mean = {:.0?}N.m ; std = {:.0?}N.m",
            moment_mean,
            moment_std
        );

        let mut data: Option<Vec<f64>> = if let Some(fm) = fm {
            Some(fm.into_iter().filter_map(|x| x).flatten().collect())
        } else {
            None
        };
        let mut m1_loads: Option<Vec<f64>> = if let Some(fm) = m1_fm {
            Some(fm.into_iter().filter_map(|x| x).flatten().collect())
        } else {
            None
        };
        let mut m2_loads: Option<Vec<f64>> = if let Some(fm) = m2_fm {
            Some(fm.into_iter().filter_map(|x| x).flatten().collect())
        } else {
            None
        };
        let n = data
            .as_ref()
            .map_or(m1_loads.as_ref().map_or(0, |x| x.len()), |x| x.len())
            / monitors.time.len();
        if n_sample > monitors.len() {
            if let Some(ref mut data) = data {
                let mut v = data.clone();
                while n_sample * n > v.len() {
                    v = v
                        .chunks(n)
                        .chain(v.chunks(n).rev().skip(1))
                        .take(n_sample)
                        .flat_map(|x| x.to_vec())
                        .collect();
                }
                mem::swap(data, &mut v);
            }
            if let Some(ref mut data) = m1_loads {
                let mut v = data.clone();
                let n = 42;
                while n_sample * n > v.len() {
                    v = v
                        .chunks(n)
                        .chain(v.chunks(n).rev().skip(1))
                        .take(n_sample)
                        .flat_map(|x| x.to_vec())
                        .collect();
                }
                mem::swap(data, &mut v);
            }
            if let Some(ref mut data) = m2_loads {
                let mut v = data.clone();
                let n = 42;
                while n_sample * n > v.len() {
                    v = v
                        .chunks(n)
                        .chain(v.chunks(n).rev().skip(1))
                        .take(n_sample)
                        .flat_map(|x| x.to_vec())
                        .collect();
                }
                mem::swap(data, &mut v);
            }
        }
        Ok(CfdLoads {
            oss: data,
            m1: m1_loads,
            m2: m2_loads,
            nodes: self.nodes,
            n_fm: n,
            step: 0,
            upsampling: self.upsampling,
            max_step: usize::MAX,
        })
    }
}
