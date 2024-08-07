use std::sync::Arc;

use gmt_dos_actors::prelude::*;
use gmt_dos_clients::{
    interface::{Data, Read, Size, Update, Write},
    Logging, Signal, Signals,
};
use gmt_dos_clients_io::gmt_m1::{
    segment::{
        ActuatorAppliedForces, ActuatorCommandForces, BarycentricForce, HardpointsForces,
        HardpointsMotion, RBM,
    },
    M1RigidBodyMotions,
};
use gmt_dos_clients_m1_ctrl::{Actuators, Hardpoints, LoadCells};
use gmt_fem::{fem_io::OSSM1Lcl, FEM};
use matio_rs::MatFile;
use nalgebra as na;

pub struct Plant {
    gain: na::DMatrix<f64>,
    u: na::DVector<f64>,
    y: na::DVector<f64>,
}
impl Plant {
    pub fn new(gain: na::DMatrix<f64>) -> Self {
        let (n, m) = gain.shape();
        Self {
            gain,
            u: na::DVector::from_element(m, 0f64),
            y: na::DVector::from_element(n, 0f64),
        }
    }
}
impl Update for Plant {
    fn update(&mut self) {
        self.y = &self.gain * &self.u;
    }
}
impl<const ID: u8> Read<HardpointsForces<ID>> for Plant {
    fn read(&mut self, data: Arc<Data<HardpointsForces<ID>>>) {
        let n = self.u.len();
        self.u.as_mut_slice()[n - 6..].copy_from_slice(&**data);
    }
}
impl<const ID: u8> Read<ActuatorAppliedForces<ID>> for Plant {
    fn read(&mut self, data: Arc<Data<ActuatorAppliedForces<ID>>>) {
        let n = self.u.len();
        self.u.as_mut_slice()[..n - 6].copy_from_slice(&**data);
    }
}
impl<const ID: u8> Write<HardpointsMotion<ID>> for Plant {
    fn write(&mut self) -> Option<Arc<Data<HardpointsMotion<ID>>>> {
        Some(Arc::new(Data::new(self.y.as_slice()[..12].to_vec())))
    }
}
impl Write<M1RigidBodyMotions> for Plant {
    fn write(&mut self) -> Option<Arc<Data<M1RigidBodyMotions>>> {
        Some(Arc::new(Data::new(self.y.as_slice()[12..].to_vec())))
    }
}

const ACTUATOR_RATE: usize = 100;

macro_rules! segment_model {
    ($sid:expr) => {
        let sim_sampling_frequency = 1000;
        let sim_duration = 30_usize; // second
        let n_step = sim_sampling_frequency * sim_duration;

        let matfile = MatFile::load("/home/rconan/projects/m1-ctrl/calib_dt/m1_ctrl_dt.mat")?;
        let m1_hpk: f64 = matfile.var("m1_HPk")?;

        let mut fem = FEM::from_env()?;
        // println!("{fem}");

        fem.keep_inputs(&[$sid.into(), 15]);
        fem.filter_inputs_by(&[15], |x| {
            x.descriptions.contains(&format!("M1-S{} hardpoint", $sid))
        });
        fem.keep_outputs(&[22, 24]);
        fem.filter_outputs_by(&[22], |x| {
            x.descriptions.contains(&format!("M1-S{} hardpoint", $sid))
        });
        fem.filter_outputs_by(&[24], |x| x.descriptions.contains(&format!("M1-S{}", $sid)));
        println!("{fem}");

        let static_gain = fem.reduced_static_gain().unwrap();

        let rbm_2_hp = {
            let rbm_2_hp: Vec<f64> = matfile.var(format!("S{}_M1RBM2HP", $sid))?;
            na::Matrix6::from_column_slice(rbm_2_hp.as_slice())
        };
        let lc_2_cg = {
            let lc_2_cg: na::DMatrix<f64> = if $sid == 7 {
                matfile.var("CS_LC2CG")
            } else {
                matfile.var("OA_LC2CG")
            }?;
            na::Matrix6::from_column_slice(lc_2_cg.as_slice())
        };

        let rbm_fun = |i| (-1f64).powi(i as i32) * (1 + (i % 3)) as f64;
        let mut hp_setpoint: Initiator<_> = (
            (0..6).fold(Signals::new(6, n_step), |signals, i| {
                signals.channel(i, Signal::Constant(rbm_fun(i) * 1e-6))
            }),
            "RBM",
        )
            .into();
        let mut hardpoints: Actor<_> = Hardpoints::new(m1_hpk, rbm_2_hp).into();

        let mut loadcell: Actor<_, 1, ACTUATOR_RATE> = LoadCells::new(m1_hpk, lc_2_cg).into();

        let mut actuators: Actor<_, ACTUATOR_RATE, 1> = Actuators::<$sid>::new().into();
        let mut actuators_setpoint: Initiator<_, ACTUATOR_RATE> = (
            Signals::new(
                Size::<ActuatorCommandForces<$sid>>::len(&Actuators::<$sid>::new()),
                n_step,
            ),
            "Actuators",
        )
            .into();

        let mut plant: Actor<_> = Plant::new(static_gain).into();

        // let logging = Logging::<f64>::new(1).into_arcx();
        // let mut logger: Terminator<_> = Actor::new(logging.clone());

        // let a_logging = Logging::<f64>::new(1).into_arcx();
        // let mut a_logger: Terminator<_, 100> = Actor::new(a_logging.clone());

        let plant_logging = Logging::<f64>::new(1).into_arcx();
        let mut plant_logger: Terminator<_> = Actor::new(plant_logging.clone());

        hp_setpoint
            .add_output()
            .build::<RBM<$sid>>()
            .into_input(&mut hardpoints)?;

        actuators_setpoint
            .add_output()
            .build::<ActuatorCommandForces<$sid>>()
            .into_input(&mut actuators)?;

        hardpoints
            .add_output()
            .multiplex(2)
            .build::<HardpointsForces<$sid>>()
            .into_input(&mut loadcell)
            .into_input(&mut plant)?;
        // .into_input(&mut logger);

        loadcell
            .add_output()
            .bootstrap()
            .build::<BarycentricForce<$sid>>()
            .into_input(&mut actuators)?;
        // .into_input(&mut a_logger);

        actuators
            .add_output()
            .build::<ActuatorAppliedForces<$sid>>()
            .into_input(&mut plant)?;

        plant
            .add_output()
            .bootstrap()
            .build::<HardpointsMotion<$sid>>()
            .into_input(&mut loadcell)?;

        plant
            .add_output()
            .bootstrap()
            .build::<M1RigidBodyMotions>()
            .into_input(&mut plant_logger)?;

        model!(
            hp_setpoint,
            actuators_setpoint,
            hardpoints,
            loadcell,
            actuators,
            plant,
            // logger,
            // a_logger,
            plant_logger
        )
        .flowchart()
        .check()?
        .run()
        .await?;

        /*     println!("HardpointsForces");
        (*logging.lock().await)
            .chunks()
            .enumerate()
            .skip(n_step - 20)
            .for_each(|(i, x)| println!("{:4}: {:+.3?}", i, x));

        println!("BarycentricForce");
        (*a_logging.lock().await)
            .chunks()
            .enumerate()
            .skip(n_step / ACTUATOR_RATE - 20)
            .for_each(|(i, x)| println!("{:4}: {:+.3?}", i, x));

        println!("Plant HardpointsMotion & M1 S1 RBM");
        (*plant_logging.lock().await)
            .chunks()
            .enumerate()
            .skip(n_step - 20)
            .map(|(i, x)| (i, x.iter().map(|x| x * 1e6).collect::<Vec<f64>>()))
            .for_each(|(i, x)| println!("{:4}: {:+.3?}", i, x)); */

        let rbm_err = ((*plant_logging.lock().await)
            .chunks()
            .last()
            .unwrap()
            .iter()
            .enumerate()
            .map(|(i, x)| x * 1e6 - rbm_fun(i))
            .map(|x| x * x)
            .sum::<f64>()
            / 6f64)
            .sqrt();

        assert!(dbg!(rbm_err) < 1e-3);
    };
}

#[tokio::test]
async fn segment() -> anyhow::Result<()> {
    {
        const S1: u8 = 1;
        segment_model!(S1);
    }
    {
        const S2: u8 = 2;
        segment_model!(S2);
    }
    {
        const S3: u8 = 3;
        segment_model!(S3);
    }
    {
        const S4: u8 = 4;
        segment_model!(S4);
    }
    {
        const S5: u8 = 5;
        segment_model!(S5);
    }
    {
        const S6: u8 = 6;
        segment_model!(S6);
    }
    {
        const S7: u8 = 7;
        segment_model!(S7);
    }

    Ok(())
}
