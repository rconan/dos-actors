use std::collections::HashMap;

use geotrans::{Quaternion, Vector};
use gmt_fem::FEM;

use crate::StateSpaceError;

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Debug, Clone, Default)]
pub struct RbmRemoval(HashMap<u8, Vec<f64>>);

impl RbmRemoval {
    pub fn new(fem: &Box<FEM>, output_template: &str) -> Result<Self, StateSpaceError> {
        let mut map: HashMap<u8, Vec<f64>> = HashMap::new();
        for i in 1..=7 {
            // let output_name = format!("M2_segment_{i}_axial_d");
            let output_name = output_template.replace("#", i.to_string().as_str());
            // println!("Loading nodes from {output_name}");
            let idx =
                Box::<dyn crate::fem_io::GetOut>::try_from(output_name.clone()).map(|x| {
                    x.position(&fem.outputs)
                        .ok_or(StateSpaceError::IndexNotFound(output_name.clone()))
                })??;
            let xyz = fem.outputs[idx]
                .as_ref()
                .map(|i| i.get_by(|i| i.properties.location.clone()))
                .expect(&format!(
                    "failed to read nodes locations from {output_name}"
                ))
                .into_iter()
                .flatten()
                .collect();
            map.insert(i as u8, xyz);
        }
        Ok(Self(map))
    }
    pub fn from_segment(&mut self, sid: u8, figure: &[f64], rbms: &[f64]) -> Option<Vec<f64>> {
        rbms.chunks(6)
            .nth(sid as usize - 1)
            .zip(self.0.get_mut(&sid))
            .map(|(rbm, nodes)| Self::rbm_removal(rbm, nodes, figure))
    }
    pub fn from_assembly(
        &mut self,
        sids: impl Iterator<Item = u8>,
        data: &[Vec<f64>],
        rbms: &[f64],
    ) -> Option<Vec<Vec<f64>>> {
        data.iter()
            .zip(sids)
            .map(|(figure, id)| self.from_segment(id, figure, rbms))
            .collect()
    }
    fn rbm_removal(rbm: &[f64], nodes: &mut [f64], figure: &[f64]) -> Vec<f64> {
        let tz = rbm[2];
        let q = Quaternion::unit(rbm[5], Vector::k())
            * Quaternion::unit(rbm[4], Vector::j())
            * Quaternion::unit(rbm[3], Vector::i());
        nodes
            .chunks_mut(3)
            .zip(figure)
            .map(|(u, dz)| {
                u[2] = dz - tz;
                let p: Quaternion = From::<&[f64]>::from(u);
                let pp = q.complex_conjugate() * p * &q;
                let v: Vec<f64> = pp.vector_as_slice().to_vec();
                v[2]
            })
            .collect()
    }
}
