use crate::{
    ngao::DetectorFrame,
    sensors::{NoSensor, SensorPropagation},
    OpticalModelBuilder,
};
use crseo::{Atmosphere, FromBuilder, Gmt, SegmentWiseSensor, Source};
use gmt_dos_clients_io::{
    gmt_m1::{
        assembly::M1ModeCoefficients,
        segment::{ModeShapes, RBM},
        M1ModeShapes, M1RigidBodyMotions,
    },
    gmt_m2::{
        asm::{
            segment::{AsmCommand, FaceSheetFigure},
            M2ASMAsmCommand, M2ASMFaceSheetFigure,
        },
        M2RigidBodyMotions,
    },
    optics::{
        M1GlobalTipTilt, M1Modes, M1State, M2GlobalTipTilt, M2Modes, M2State, SegmentD7Piston,
    },
};
use interface::{Data, Read, UniqueIdentifier, Units, Update, Write};

pub mod builder;
mod imaging;
mod pyramid;
mod stats;

#[derive(Debug, thiserror::Error)]
pub enum OpticalModelError {
    #[error("failed to build optical model")]
    Crseo(#[from] crseo::error::CrseoError),
    #[error("atmosphere is set but not the sampling frequency")]
    AtmosphereWithoutSamplingFrequency,
    #[error("no sensor has been set")]
    MissingSensor,
}

/// GMT optical model
///
/// GMT M1 and M2 optical prescriptions.
/// May as well include a [sensor](crate::sensors) and a model of the [atmospheric turbulence](https://docs.rs/crseo/latest/crseo/atmosphere).
///
/// ## Example
///
/// Build a optical model with the default [OpticalModelBuilder]
/// ```
/// use gmt_dos_clients_crseo::{OpticalModel, sensors::NoSensor};
///
/// let om = OpticalModel::<NoSensor>::builder().build()?;
/// # Ok::<(),Box<dyn std::error::Error>>(())
/// ```
pub struct OpticalModel<T = NoSensor> {
    pub(crate) gmt: Gmt,
    pub(crate) src: Source,
    pub(crate) atm: Option<Atmosphere>,
    pub(crate) sensor: Option<T>,
    pub(crate) tau: f64,
    pub(crate) phase_offset: Option<Vec<f64>>,
}

impl<T> OpticalModel<T> {
    /// Returns a mutable reference to the sensor
    pub fn sensor_mut(&mut self) -> Option<&mut T> {
        self.sensor.as_mut()
    }
    /// Returns an immutable reference to the sensor
    pub fn sensor(&self) -> Option<&T> {
        self.sensor.as_ref()
    }
    /// Returns a mutable reference to the source
    pub fn source(&self) -> &Source {
        &self.src
    }
    /// Returns a immutable reference to the source
    pub fn source_mut(&mut self) -> &mut Source {
        &mut self.src
    }
    pub fn phase_offset(&mut self, phase_offset: &[f64]) -> &mut Self {
        self.phase_offset = Some(phase_offset.to_vec());
        self
    }
}
unsafe impl<T> Send for OpticalModel<T> {}
unsafe impl<T> Sync for OpticalModel<T> {}

impl<T> Units for OpticalModel<T> {}

impl<T> OpticalModel<T>
where
    T: FromBuilder,
{
    /// Creates an optical model builder
    pub fn builder() -> OpticalModelBuilder<<T as FromBuilder>::ComponentBuilder> {
        let OpticalModelBuilder {
            gmt,
            src,
            atm_builder,
            sampling_frequency,
            ..
        } = OpticalModelBuilder::<NoSensor>::default();
        OpticalModelBuilder {
            gmt,
            src,
            atm_builder,
            sensor: Some(T::builder()),
            sampling_frequency,
        }
    }
}

impl<T: SensorPropagation> Update for OpticalModel<T> {
    fn update(&mut self) {
        self.src.through(&mut self.gmt).xpupil();
        if let Some(atm) = &mut self.atm {
            atm.secs += self.tau;
            self.src.through(atm);
        }
        if let Some(phase) = self.phase_offset.as_ref() {
            self.src.add(phase);
        }
        if let Some(sensor) = &mut self.sensor {
            sensor.propagate(&mut self.src);
        }
    }
}

impl<T: SensorPropagation, const SID: u8> Read<RBM<SID>> for OpticalModel<T> {
    fn read(&mut self, data: Data<RBM<SID>>) {
        self.gmt
            .m1_segment_state(SID as i32, &data[..3], &data[3..]);
    }
}

impl<T: SensorPropagation> Read<M1RigidBodyMotions> for OpticalModel<T> {
    fn read(&mut self, data: Data<M1RigidBodyMotions>) {
        data.chunks(6).enumerate().for_each(|(sid, data)| {
            self.gmt
                .m1_segment_state(1 + sid as i32, &data[..3], &data[3..]);
        });
    }
}

impl<T: SensorPropagation, const SID: u8> Read<ModeShapes<SID>> for OpticalModel<T> {
    fn read(&mut self, data: Data<ModeShapes<SID>>) {
        self.gmt.m1_segment_modes(SID, &data);
    }
}

impl<T: SensorPropagation> Read<M1Modes> for OpticalModel<T> {
    fn read(&mut self, data: Data<M1Modes>) {
        assert_eq!(7 * self.gmt.m1.n_mode, data.len());
        self.gmt.m1_modes(&data);
    }
}
impl<T: SensorPropagation> Read<M1State> for OpticalModel<T> {
    fn read(&mut self, data: Data<M1State>) {
        let state = data.into_arc();
        if let Some(rbms) = &state.rbms {
            <Self as Read<M1RigidBodyMotions>>::read(self, rbms.into());
        }
        if let Some(modes) = &state.modes {
            <Self as Read<M1ModeShapes>>::read(self, modes.into());
        }
    }
}
impl<T: SensorPropagation> Read<M1ModeShapes> for OpticalModel<T> {
    fn read(&mut self, data: Data<M1ModeShapes>) {
        self.gmt.m1_modes(&data);
    }
}
impl<T: SensorPropagation> Read<M1ModeCoefficients> for OpticalModel<T> {
    fn read(&mut self, data: Data<M1ModeCoefficients>) {
        self.gmt.m1_modes(&data);
    }
}

impl<T: SensorPropagation> Read<M2RigidBodyMotions> for OpticalModel<T> {
    fn read(&mut self, data: Data<M2RigidBodyMotions>) {
        data.chunks(6).enumerate().for_each(|(sid, data)| {
            self.gmt
                .m2_segment_state(1 + sid as i32, &data[..3], &data[3..]);
        });
    }
}

impl<T: SensorPropagation, const SID: u8> Read<AsmCommand<SID>> for OpticalModel<T> {
    fn read(&mut self, data: Data<AsmCommand<SID>>) {
        self.gmt.m2_segment_modes(SID, &data);
    }
}

impl<T: SensorPropagation> Read<M2ASMAsmCommand> for OpticalModel<T> {
    fn read(&mut self, data: Data<M2ASMAsmCommand>) {
        self.gmt.m2_modes(&data);
    }
}

impl<T: SensorPropagation> Read<M1GlobalTipTilt> for OpticalModel<T> {
    fn read(&mut self, data: Data<M1GlobalTipTilt>) {
        let rbms = geotrans::Mirror::<geotrans::M1>::tiptilt_2_rigidbodymotions((data[0], data[1]));
        <OpticalModel<T> as Read<M1RigidBodyMotions>>::read(self, rbms.into())
    }
}
impl<T: SensorPropagation> Read<M2GlobalTipTilt> for OpticalModel<T> {
    fn read(&mut self, data: Data<M2GlobalTipTilt>) {
        let rbms = geotrans::Mirror::<geotrans::M2>::tiptilt_2_rigidbodymotions((data[0], data[1]));
        /* rbms.chunks(6).enumerate().for_each(|(i, c)| {
            println!(
                "S{}, {:+7.0?} {:+7.0?}",
                i + 1,
                c[..3].iter().map(|x| x * 1e9).collect::<Vec<_>>(),
                c[3..].iter().map(|x| x.to_mas()).collect::<Vec<_>>()
            )
        }); */
        <OpticalModel<T> as Read<M2RigidBodyMotions>>::read(self, rbms.into())
    }
}

// impl<T: SensorPropagation> Read<M2Modes> for OpticalModel<T> {
//     fn read(&mut self, data: Data<M2Modes>) {
//         assert_eq!(7 * self.gmt.m2.n_mode, data.len());
//         // if 7 * self.gmt.m2.n_mode > data.len() {
//         //     let augmented_data: Vec<_> = data
//         //         .chunks(data.len() / 7)
//         //         .flat_map(|data| {
//         //             let mut v = vec![0f64];
//         //             v.extend_from_slice(data);
//         //             v
//         //         })
//         //         .collect();
//         //     assert_eq!(augmented_data.len(), self.gmt.m2.n_mode * 7);
//         //     self.gmt.m2_modes(&augmented_data);
//         // } else {
//         self.gmt.m2_modes(&data);
//         // }
//     }
// }
impl<T: SensorPropagation> Read<M2Modes> for OpticalModel<T> {
    fn read(&mut self, data: Data<M2Modes>) {
        // assert_eq!(7 * self.gmt.m2.n_mode, data.len());
        if 7 * self.gmt.m2.n_mode > data.len() {
            let augmented_data: Vec<_> = data
                .chunks(data.len() / 7)
                .flat_map(|data| {
                    let mut v = vec![0f64];
                    v.extend_from_slice(data);
                    v
                })
                .collect();
            assert_eq!(augmented_data.len(), self.gmt.m2.n_mode * 7);
            self.gmt.m2_modes(&augmented_data);
        } else {
            self.gmt.m2_modes(&data);
        }
    }
}

impl<T: SensorPropagation> Read<M2State> for OpticalModel<T> {
    fn read(&mut self, data: Data<M2State>) {
        let state = data.into_arc();
        if let Some(rbms) = &state.rbms {
            <Self as Read<M2RigidBodyMotions>>::read(self, rbms.into());
        }
        if let Some(modes) = &state.modes {
            <Self as Read<M2Modes>>::read(self, modes.into());
        }
    }
}

impl<T> Write<DetectorFrame> for OpticalModel<T>
where
    T: SegmentWiseSensor,
    DetectorFrame: UniqueIdentifier<DataType = crseo::Frame>,
{
    fn write(&mut self) -> Option<Data<DetectorFrame>> {
        self.sensor.as_mut().map(|sensor| {
            let frame = SegmentWiseSensor::frame(sensor);
            <T as crseo::WavefrontSensor>::reset(sensor);
            Data::new(frame)
        })
    }
}

impl<T: SegmentWiseSensor, const E: i32> Write<SegmentD7Piston<E>> for OpticalModel<T> {
    fn write(&mut self) -> Option<Data<SegmentD7Piston<E>>> {
        let data = self.src.segment_wfe();
        let p7 = data[6].0;
        // let data = &self.segment_wfe;
        Some(
            data.into_iter()
                .map(|(p, _)| (p - p7) * 10_f64.powi(-E))
                .collect::<Vec<_>>()
                .into(),
        )
    }
}

impl<T: SensorPropagation, const ID: u8> Read<FaceSheetFigure<ID>> for OpticalModel<T> {
    fn read(&mut self, data: Data<FaceSheetFigure<ID>>) {
        self.gmt.m2_segment_modes(ID, &data);
    }
}
impl<T: SensorPropagation> Read<M2ASMFaceSheetFigure> for OpticalModel<T> {
    fn read(&mut self, data: Data<M2ASMFaceSheetFigure>) {
        let q: Vec<_> = data.iter().flatten().cloned().collect();
        self.gmt.m2_modes(q.as_slice());
    }
}
