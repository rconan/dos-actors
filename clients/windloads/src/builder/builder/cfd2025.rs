use super::Builder;
use crate::{CS, CfdLoads, M1S, M2S, MAX_DURATION, Result};
use geotrans::{SegmentTrait, Transform};
use parse_monitors::{Exertion, Monitors, Vector};
use std::mem;

impl<S> Builder<S> {
    /// Returns a [CfdLoads] object
    pub fn build(self) -> Result<CfdLoads<S>> {
        // println!("Loading the CFD loads from {} ...", self.cfd_case);
        // let now = Instant::now();
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
