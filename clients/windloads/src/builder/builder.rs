use crate::{
    CS,
    windloads::{WindLoads, WindLoadsBuilder},
};
use serde::{Deserialize, Serialize};

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
mod cfd2021;

#[cfg(any(
    cfd2025,
    all(feature = "cfd2025", not(cfd2021), not(feature = "cfd2021"))
))]
mod cfd2025;
