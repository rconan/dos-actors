use gmt_dos_actors::system::SystemError;
pub use gmt_dos_clients_m1_ctrl::Calibration;
use gmt_fem::FEM;

pub mod assembly;
pub mod segment_control;
pub mod systems;

pub enum M1<const ACTUATOR_RATE: usize> {}
impl<const ACTUATOR_RATE: usize> M1<ACTUATOR_RATE> {
    pub fn new(
        fem: &mut FEM,
    ) -> Result<gmt_dos_actors::system::Sys<assembly::M1<ACTUATOR_RATE>>, SystemError> {
        let calibration = Calibration::new(fem);
        Ok(
            gmt_dos_actors::system::Sys::new(assembly::M1::<ACTUATOR_RATE>::new(&calibration)?)
                .build()?,
        )
    }
}

#[derive(Debug, Clone, Default)]
pub struct M1Builder<const ACTUATOR_RATE: usize> {
    calibration: Calibration,
    mode_2_force_transforms: Option<Vec<nalgebra::DMatrix<f64>>>,
}
impl<const ACTUATOR_RATE: usize> M1Builder<ACTUATOR_RATE> {
    pub fn modes_to_forces(mut self, transforms: Vec<nalgebra::DMatrix<f64>>) -> Self {
        self.mode_2_force_transforms = Some(transforms);
        self
    }
}

impl<const ACTUATOR_RATE: usize> M1<ACTUATOR_RATE> {
    pub fn builder(fem: &mut FEM) -> M1Builder<ACTUATOR_RATE> {
        let calibration = Calibration::new(fem);

        M1Builder {
            calibration,
            ..Default::default()
        }
    }
}

// impl<const ACTUATOR_RATE: usize> M1Builder<ACTUATOR_RATE> {
//     /// Builds the [ASMS] system
//     pub fn build(self) -> Result<Sys<assembly::M1<ACTUATOR_RATE>>, SystemError> {
//         Ok(Sys::new(assembly::M1::<ACTUATOR_RATE>::try_from(self)?).build()?)
//     }
// }

// impl<const ACTUATOR_RATE: usize> TryFrom<M1Builder<ACTUATOR_RATE>>
//     for Sys<assembly::M1<ACTUATOR_RATE>>
// {
//     type Error = SystemError;

//     fn try_from(builder: M1Builder<ACTUATOR_RATE>) -> std::result::Result<Self, Self::Error> {
//         Sys::new(builder.build()?).build()
//     }
// }
