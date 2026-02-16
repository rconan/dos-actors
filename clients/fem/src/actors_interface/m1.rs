//! M1 CONTROL

#[doc(hidden)]
pub use super::prelude;
use super::prelude::*;
use gmt_dos_clients_io::{
    Assembly,
    gmt_m1::{M1EdgeSensors, M1ModeShapes, M1RigidBodyMotions, segment::ModeShapes},
    optics::{M1State, state::SegmentState},
};

pub mod actuators;
pub mod assembly;
pub mod hardpoints;
pub mod rigid_body_motions;

/* impl<S> Get<M1ModeShapes> for DiscreteModalSolver<S>
where
    S: Solver + Default,
{
    fn get(&self) -> Option<Vec<f64>> {
        let mut encoders = <DiscreteModalSolver<S> as Get<fem_io::M1Segment1AxialD>>::get(self)?;
        encoders.extend(
            <DiscreteModalSolver<S> as Get<fem_io::M1Segment2AxialD>>::get(self)?.as_slice(),
        );
        encoders.extend(
            <DiscreteModalSolver<S> as Get<fem_io::M1Segment3AxialD>>::get(self)?.as_slice(),
        );
        encoders.extend(
            <DiscreteModalSolver<S> as Get<fem_io::M1Segment4AxialD>>::get(self)?.as_slice(),
        );
        encoders.extend(
            <DiscreteModalSolver<S> as Get<fem_io::M1Segment5AxialD>>::get(self)?.as_slice(),
        );
        encoders.extend(
            <DiscreteModalSolver<S> as Get<fem_io::M1Segment6AxialD>>::get(self)?.as_slice(),
        );
        encoders.extend(
            <DiscreteModalSolver<S> as Get<fem_io::M1Segment7AxialD>>::get(self)?.as_slice(),
        );
        Some(encoders)
    }
} */
impl<S> Write<M1ModeShapes> for DiscreteModalSolver<S>
where
    DiscreteModalSolver<S>: Iterator,
    S: Solver + Default,
{
    fn write(&mut self) -> Option<Data<M1ModeShapes>> {
        // let data: Vec<_> = <M1ModeShapes as Assembly>::SIDS
        //     .into_iter()
        //     .filter_map(|sid| match sid {
        //         1 => <DiscreteModalSolver<S> as Get<fem_io::M1Segment1AxialD>>::get(self),
        //         2 => <DiscreteModalSolver<S> as Get<fem_io::M1Segment2AxialD>>::get(self),
        //         3 => <DiscreteModalSolver<S> as Get<fem_io::M1Segment3AxialD>>::get(self),
        //         4 => <DiscreteModalSolver<S> as Get<fem_io::M1Segment4AxialD>>::get(self),
        //         5 => <DiscreteModalSolver<S> as Get<fem_io::M1Segment5AxialD>>::get(self),
        //         6 => <DiscreteModalSolver<S> as Get<fem_io::M1Segment6AxialD>>::get(self),
        //         7 => <DiscreteModalSolver<S> as Get<fem_io::M1Segment7AxialD>>::get(self),
        //         _ => panic!("expected segment id with [1,7], found {:}", sid),
        //     })
        //     .collect();
        // if self.m1_figure_nodes.is_some() {
        //     let rbms = <DiscreteModalSolver<S> as Get<fem_io::OSSM1Lcl>>::get(self)
        //         .expect("failed to get rigid body motion from ASMS reference bodies");
        //     self.m1_figure_nodes.as_mut().map(|m1_figure| {
        //         m1_figure
        //             .from_assembly(<M1ModeShapes as Assembly>::SIDS.into_iter(), &data, &rbms)
        //             .expect("failed to remove RBM from ASM m1_figure")
        //     })
        // } else {
        //     Some(data)
        // }
        // .inspect(|x| {
        //     dbg!(x[0].len());
        // })
        // .map(|x| { x.into_iter().flatten().collect::<Vec<_>>() }.into())
        <_ as Write<M1State>>::write(self)
            .map(|data| {
                data.iter()
                    .filter_map(|segment| {
                        if let Some(SegmentState {
                            modes: Some(modes), ..
                        }) = segment
                        {
                            Some(modes.as_ref().to_vec())
                        } else {
                            None
                        }
                    })
                    .inspect(|x| {
                    })
                    .flatten()
                    .collect::<Vec<_>>()
            })
            .map(|x| x.into())
    }
}
impl<S> Write<M1State> for DiscreteModalSolver<S>
where
    DiscreteModalSolver<S>: Iterator,
    S: Solver + Default,
{
    fn write(&mut self) -> Option<Data<M1State>> {
        // M1 RBMs (42)
        let rbms = <DiscreteModalSolver<S> as Get<fem_io::OSSM1Lcl>>::get(self);
        // Segment RBMS (6x7)
        let mut segment_rbms = rbms.as_ref().map(|rbms| rbms.chunks(6));
        // M1 shape matrix transforms ([n_mode,n_node]x7)
        let transforms = self.m1_figure_transforms.as_ref();
        // M1 shape matrix transforms ([n_mode,n_node]x7) iterator
        let mut segment_transforms = transforms.map(|transforms| transforms.iter());

        // let state: MirrorState = <M1ModeShapes as Assembly>::SIDS
        //     .into_iter()
        // .map(|sid| {
        let mut segments = vec![];
        for sid in <M1ModeShapes as Assembly>::SIDS.into_iter() {
            // segment # sid RBMS
            let rbms = segment_rbms.as_mut().map(|rbms| rbms.next());
            // segment # sid shape transforms
            let transforms = segment_transforms
                .as_mut()
                .map(|transforms| transforms.next());
            // segment figure (aka shape)
            let figure = match sid {
                1 => <DiscreteModalSolver<S> as Get<fem_io::M1Segment1AxialD>>::get(self),
                2 => <DiscreteModalSolver<S> as Get<fem_io::M1Segment2AxialD>>::get(self),
                3 => <DiscreteModalSolver<S> as Get<fem_io::M1Segment3AxialD>>::get(self),
                4 => <DiscreteModalSolver<S> as Get<fem_io::M1Segment4AxialD>>::get(self),
                5 => <DiscreteModalSolver<S> as Get<fem_io::M1Segment5AxialD>>::get(self),
                6 => <DiscreteModalSolver<S> as Get<fem_io::M1Segment6AxialD>>::get(self),
                7 => <DiscreteModalSolver<S> as Get<fem_io::M1Segment7AxialD>>::get(self),
                _ => panic!("expected segment id with [1,7], found {:}", sid),
            };
            let segment = match (rbms, figure) {
                (None, Some(mut figure)) | (Some(None), Some(mut figure)) => {
                    if let Some(Some(mat)) = transforms {
                        let y = mat * nalgebra::DVector::from_column_slice(&figure);
                        let y_slice = y.as_slice();
                        figure.resize(y_slice.len(), Default::default());
                        figure.copy_from_slice(y_slice);
                    }
                    SegmentState::modes(figure)
                }
                (Some(Some(rbms)), None) => SegmentState::rbms(rbms),
                (Some(Some(rbms)), Some(mut figure)) => {
                    self.m1_figure_nodes.as_mut().map(|m1_figure| {
                        m1_figure.rbms_removal(sid, &mut figure, rbms);
                    });
                    if let Some(Some(mat)) = transforms {
                        let y = mat * nalgebra::DVector::from_column_slice(&figure);
                        let y_slice = y.as_slice();
                        figure.resize(y_slice.len(), Default::default());
                        figure.copy_from_slice(y_slice);
                    }
                    SegmentState::new(rbms, figure)
                }
                _ => SegmentState::default(),
            };
            segments.push(segment);
        }
        // .collect();

        // let data: Vec<_> = <M1ModeShapes as Assembly>::SIDS
        //     .into_iter()
        //     .filter_map(|sid| match sid {
        //         1 => <DiscreteModalSolver<S> as Get<fem_io::M1Segment1AxialD>>::get(self),
        //         2 => <DiscreteModalSolver<S> as Get<fem_io::M1Segment2AxialD>>::get(self),
        //         3 => <DiscreteModalSolver<S> as Get<fem_io::M1Segment3AxialD>>::get(self),
        //         4 => <DiscreteModalSolver<S> as Get<fem_io::M1Segment4AxialD>>::get(self),
        //         5 => <DiscreteModalSolver<S> as Get<fem_io::M1Segment5AxialD>>::get(self),
        //         6 => <DiscreteModalSolver<S> as Get<fem_io::M1Segment6AxialD>>::get(self),
        //         7 => <DiscreteModalSolver<S> as Get<fem_io::M1Segment7AxialD>>::get(self),
        //         _ => panic!("expected segment id with [1,7], found {:}", sid),
        //     })
        //     .collect();
        // let rbms = <DiscreteModalSolver<S> as Get<fem_io::OSSM1Lcl>>::get(self)
        //     .expect("failed to get M1 rigid body motions");
        // let state = if data.is_empty() {
        //     MirrorState::from_rbms(&rbms)
        // } else {
        //     let modes = if self.m1_figure_nodes.is_some() {
        //         self.m1_figure_nodes.as_mut().map(|m1_figure| {
        //             m1_figure
        //                 .from_assembly(<M1ModeShapes as Assembly>::SIDS.into_iter(), &data, &rbms)
        //                 .expect("failed to remove RBM from ASM m1_figure")
        //         })
        //     } else {
        //         Some(data)
        //     }
        //     .map(|x| {
        //         if let Some(transforms) = &self.m1_figure_transforms {
        //             Box::new(x.into_iter().zip(transforms).map(|(x, mat)| {
        //                 let y = mat * nalgebra::DVector::from_column_slice(&x);
        //                 y.as_slice().to_vec()
        //             }))
        //         } else {
        //             Box::new(x.into_iter()) as Box<dyn Iterator<Item = Vec<f64>>>
        //         }
        //     });
        //     let rbms_chunks = rbms.chunks(6);
        //     let state: MirrorState = match modes {
        //         Some(modes) => rbms_chunks
        //             .zip(modes)
        //             .map(|(rbms, modes)| SegmentState::new(rbms.to_vec(), modes))
        //             .collect(),
        //         None => rbms_chunks
        //             .map(|rbms| SegmentState::rbms(rbms.to_vec()))
        //             .collect(),
        //     };
        //     state
        // };
        Some(Data::new(segments.into_iter().collect()))
    }
}
impl<const ID: u8, S> Write<ModeShapes<ID>> for DiscreteModalSolver<S>
where
    DiscreteModalSolver<S>: Iterator,
    S: Solver + Default,
{
    fn write(&mut self) -> Option<Data<ModeShapes<ID>>> {
        let mut figure = match ID {
            1 => <DiscreteModalSolver<S> as Get<fem_io::M1Segment1AxialD>>::get(self),
            2 => <DiscreteModalSolver<S> as Get<fem_io::M1Segment2AxialD>>::get(self),
            3 => <DiscreteModalSolver<S> as Get<fem_io::M1Segment3AxialD>>::get(self),
            4 => <DiscreteModalSolver<S> as Get<fem_io::M1Segment4AxialD>>::get(self),
            5 => <DiscreteModalSolver<S> as Get<fem_io::M1Segment5AxialD>>::get(self),
            6 => <DiscreteModalSolver<S> as Get<fem_io::M1Segment6AxialD>>::get(self),
            7 => <DiscreteModalSolver<S> as Get<fem_io::M1Segment7AxialD>>::get(self),
            _ => unreachable!(),
        }?;
        if self.m1_figure_nodes.is_some() {
            let rbms = <DiscreteModalSolver<S> as Get<fem_io::OSSM1Lcl>>::get(self)
                .expect("failed to get rigid body motion from M1 segments");
            let y = self
                .m1_figure_nodes
                .as_mut()
                .unwrap()
                .from_segment(ID, &figure, &rbms)
                .expect("failed to remove RBM from M1 segment #{ID}");
            figure.copy_from_slice(&y);
        }
        if let Some(transforms) = &self.m1_figure_transforms {
            let mat = &transforms[ID as usize - 1];
            let y = mat * nalgebra::DVector::from_column_slice(&figure);
            let y_slice = y.as_slice();
            figure.resize(y_slice.len(), Default::default());
            figure.copy_from_slice(y_slice);
        };
        Some(figure.into())
    }
}

//  * M1 rigid body motions
impl<S> Size<M1RigidBodyMotions> for DiscreteModalSolver<S>
where
    DiscreteModalSolver<S>: Iterator,
    S: Solver + Default,
{
    fn len(&self) -> usize {
        42
    }
}
impl<S> Write<M1RigidBodyMotions> for DiscreteModalSolver<S>
where
    DiscreteModalSolver<S>: Iterator,
    S: Solver + Default,
{
    fn write(&mut self) -> Option<Data<M1RigidBodyMotions>> {
        <DiscreteModalSolver<S> as Get<fem_io::OSSM1Lcl>>::get(self).map(|data| Data::new(data))
    }
}
impl<S> Write<M1EdgeSensors> for DiscreteModalSolver<S>
where
    DiscreteModalSolver<S>: Iterator,
    S: Solver + Default,
{
    fn write(&mut self) -> Option<Data<M1EdgeSensors>> {
        <DiscreteModalSolver<S> as Get<fem_io::OSSM1EdgeSensors>>::get(self)
            .map(|data| Data::new(data))
    }
}

#[cfg(test)]
mod tests {
    use std::error::Error;

    use gmt_dos_clients_io::gmt_fem::outputs::{
        M1Segment1AxialD, M1Segment2AxialD, M1Segment3AxialD, M1Segment4AxialD, M1Segment5AxialD,
        M1Segment6AxialD, M1Segment7AxialD, OSSM1Lcl,
    };

    use crate::solvers::ExponentialMatrix;

    use super::*;

    #[test]
    fn m1_state() -> Result<(), Box<dyn Error>> {
        let mut ss = DiscreteModalSolver::<ExponentialMatrix>::from_env()?
            .sampling(1000.)
            .outs::<OSSM1Lcl>()
            .outs::<M1Segment1AxialD>()
            .outs::<M1Segment2AxialD>()
            .outs::<M1Segment3AxialD>()
            .outs::<M1Segment4AxialD>()
            .outs::<M1Segment5AxialD>()
            .outs::<M1Segment6AxialD>()
            .outs::<M1Segment7AxialD>()
            .build()?;

        ss.update();
        let data = <_ as Write<M1State>>::write(&mut ss);

        Ok(())
    }
}
