use std::{fs::File, time::Instant};

use clap::{Parser, ValueEnum};
use faer::Mat;
use faer_ext::IntoFaer;
use gmt_dos_clients_fem::{Model, Switch, fem_io};
use gmt_dos_systems_m1::{SegmentSingularModes, SingularModes};
use gmt_fem::FEM;

/// M1 singular modes (a.k.a. bending modes) computation
#[derive(Parser, Debug)]
#[command(version, about, long_about = None)]
struct Args {
    /// file name to save the modes to
    #[arg(short, long, default_value = "m1_singular_modes.pkl")]
    filename: Option<String>,
    /// singular modes null space
    #[arg(long,require_equals=true,default_value_t= NullSpace::Rbm, default_missing_value="RbmHp",value_enum)]
    null_space: NullSpace,
    /// Display the RBMs and hardpoints displacement values for the first 27 modes
    #[arg(long)]
    validation: bool,
    /// computes the singular modes without saving them to a file
    #[arg(long)]
    dry_run: bool,
}
#[derive(ValueEnum, Copy, Clone, Debug, PartialEq, Eq)]
enum NullSpace {
    /// M1 rigid body motions
    Rbm,
    /// M1 rigid body & hardpoints motions
    RbmHp,
}
// impl std::fmt::Display for NullSpace {
//     fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
//         self.to_possible_value()
//             .expect("no values are skipped")
//             .get_name()
//             .fmt(f)
//     }
// }

fn main() -> anyhow::Result<()> {
    let args = Args::parse();
    dbg!(&args);

    let null_space_n_mode = match args.null_space {
        NullSpace::Rbm => 6,
        NullSpace::RbmHp => 12,
    };

    // let gain: na::DMatrix<f64> = serde_pickle::from_reader(
    // &mut File::open("m1_actuators_gain.pkl")?,
    // Default::default(),
    // )?;
    println!("loading the fem ...");
    let now = Instant::now();
    let mut fem = FEM::from_env()?;
    println!("elapsed: {:}s", now.elapsed().as_secs());

    // let gain = gain.view_range(.., ..).into_faer();
    // println!("Gain matrix {:?}", gain.shape());

    // let gain_rbm = gain.subrows(gain.nrows() - NR, NR);
    // println!("RBM gain matrix {:?}", gain_rbm.shape());

    let mut m1_sms = SingularModes::default();

    for sid in 1..=7u8 {
        println!("Segment #{sid}");
        let i = (sid - 1) as usize;

        let inputs = vec![format!("M1_actuators_segment_{sid}")];
        let outputs = vec![format!("M1_segment_{sid}_axial_d")];

        println!("extracting the static gain {:?} -> {:?}", inputs, outputs);
        fem.switch_inputs(Switch::Off, None)
            .switch_outputs(Switch::Off, None);
        // M1 actuator forces to M1 surface displacement gain
        let gain_d = fem
            .switch_inputs_by_name(inputs.clone(), Switch::On)
            .and_then(|fem| fem.switch_outputs_by_name(outputs.clone(), Switch::On))
            .map(|fem| fem.reduced_static_gain().unwrap())?;
        let gain_d = gain_d.view_range(.., ..).into_faer();

        println!("extracting {:?} nodes", outputs);
        let xyz: Vec<_> = outputs
            .iter()
            .flat_map(|output| {
                let get_out = Box::<dyn fem_io::GetOut>::try_from(output.clone()).unwrap();
                let idx = get_out.position(&fem.outputs).unwrap();
                fem.outputs[idx]
                    .as_ref()
                    .map(|i| i.get_by(|i| i.properties.location.clone()))
                    .unwrap()
            })
            .collect();
        println!("extracting {:?} nodes", inputs);
        let in_xyz: Vec<_> = inputs
            .iter()
            .flat_map(|output| {
                let get_out = Box::<dyn fem_io::GetIn>::try_from(output.clone()).unwrap();
                let idx = get_out.position(&fem.inputs).unwrap();
                fem.inputs[idx]
                    .as_ref()
                    .map(|i| i.get_by(|i| i.properties.location.clone()))
                    .unwrap()
            })
            .collect();

        fem.switch_inputs(Switch::Off, None)
            .switch_outputs(Switch::Off, None);
        // M1 actuator forces to M1 RBMs gain
        let gain_rbm = fem
            .switch_inputs_by_name(inputs.clone(), Switch::On)
            .and_then(|fem| fem.switch_outputs_by_name(vec!["OSS_M1_lcl"], Switch::On))
            .map(|fem| fem.reduced_static_gain().unwrap())?;
        let gain_rbm = gain_rbm.view_range(.., ..).into_faer();

        let i = (sid - 1) as usize;
        fem.switch_inputs(Switch::Off, None)
            .switch_outputs(Switch::Off, None);
        // M1 actuator forces to M1 hardpoints displacement gain
        let gain_hp = fem
            .switch_inputs_by_name(inputs.clone(), Switch::On)
            .and_then(|fem| fem.switch_outputs_by_name(vec!["OSS_Hardpoint_D"], Switch::On))
            .map(|fem| fem.reduced_static_gain().unwrap())?;
        let rows = i * 12..(i + 1) * 12;
        let gain_hp = gain_hp.view_range(rows, ..).into_faer();
        let gain_hp = gain_hp.subrows(6, 6) - gain_hp.subrows(0, 6);

        let gain_r = match args.null_space {
            NullSpace::Rbm => gain_rbm.subrows(i * 6, 6).to_owned(),
            NullSpace::RbmHp => {
                fem.switch_inputs(Switch::Off, None)
                    .switch_outputs(Switch::Off, None);
                let gain_hp = fem
                    .switch_inputs_by_name(inputs.clone(), Switch::On)
                    .and_then(|fem| fem.switch_outputs_by_name(vec!["OSS_Hardpoint_D"], Switch::On))
                    .map(|fem| fem.reduced_static_gain().unwrap())?;
                let rows = i * null_space_n_mode..(i + 1) * null_space_n_mode;
                let gain_hp = gain_hp.view_range(rows, ..).into_faer();
                let gain_hp = gain_hp.subrows(6, 6) - gain_hp.subrows(0, 6);
                let mut gain_r = Mat::<f64>::zeros(null_space_n_mode, gain_d.ncols());
                gain_r
                    .as_mut()
                    .subrows_mut(0, 6)
                    .copy_from(gain_rbm.subrows(i * 6, 6));
                gain_r
                    .as_mut()
                    .subrows_mut(6, 6)
                    .copy_from(gain_hp.as_ref());
                gain_r
            }
        };
        println!(" R gain matrix {:?}", gain_r.shape());
        let svd_r = gain_r.svd().unwrap();
        let v_r = svd_r.V().subcols(0, null_space_n_mode);
        println!(
            "  SVD R gain: U {:?}, S ({:}), V {:?}",
            svd_r.U().shape(),
            svd_r.S().column_vector().nrows(),
            v_r.shape()
        );

        println!(" D gain matrix {:?}", gain_d.shape());
        let svd_d = gain_d.svd().unwrap();
        let u_d = svd_d.U().subcols(0, svd_d.S().column_vector().nrows());
        let v_d = svd_d.V();
        let s_d = svd_d.S().column_vector();
        let mut s_d_inverse = s_d.cloned();
        s_d_inverse.iter_mut().for_each(|x| *x = x.recip());
        println!(
            "  SVD D gain: U {:?}, S ({:}), V {:?}",
            u_d.shape(),
            s_d.nrows(),
            v_d.shape()
        );

        // Projecting out RBM right eigen vectors
        let v_dr = v_d - v_r * v_r.transpose() * v_d;
        let gain_rd = u_d * svd_d.S().column_vector().as_diagonal() * &v_dr.transpose();
        let svd_rd = gain_rd.svd().unwrap();
        let u_rd = svd_rd
            .U()
            .subcols(0, svd_rd.S().column_vector().nrows() - null_space_n_mode);
        let mut s_rd = svd_rd
            .S()
            .column_vector()
            .subrows(0, svd_rd.S().column_vector().nrows() - null_space_n_mode)
            .to_owned();
        let v_rd = svd_rd
            .V()
            .subcols(0, svd_rd.S().column_vector().nrows() - null_space_n_mode);
        println!(
            "  SVD RD gain: U {:?}, S ({:}), V {:?}",
            u_rd.shape(),
            s_rd.nrows(),
            v_rd.shape()
        );

        s_rd.iter_mut().for_each(|x| *x = x.recip());

        let raw_modes = u_d.to_owned();
        let modes = u_rd.to_owned();
        let mode_2_force = v_d * s_d_inverse.as_diagonal() * u_d.transpose() * u_rd;

        /* let modes_coefs = modes.transpose() * gain_d * &mode_2_force;
        println!(
            "\nU_rd^T G_d V_rd S_rd^{{-1}} = {:5.2}",
            modes_coefs.submatrix(0, 0, 10, 10).into_nalgebra()
        );

        let rbms = gain_r * &mode_2_force;
        println!("RBMs = ");
        println!("{:5.2}", rbms.subcols(0, 10).into_nalgebra());
        println!("{:5.2}", rbms.subcols(319, 10).into_nalgebra()); */

        let sms = SegmentSingularModes::new(
            xyz,
            in_xyz,
            raw_modes
                .col_iter()
                .flat_map(|c| c.iter().cloned().collect::<Vec<_>>())
                .collect(),
            modes
                .col_iter()
                .flat_map(|c| c.iter().cloned().collect::<Vec<_>>())
                .collect(),
            mode_2_force
                .col_iter()
                .flat_map(|c| c.iter().cloned().collect::<Vec<_>>())
                .collect(),
            // s_rd.iter().cloned().collect::<Vec<_>>(),
            gain_d.shape(),
        );
        m1_sms.push(sms);
    }

    if !args.dry_run {
        let file = args.filename.unwrap_or("m1_singular_modes.pkl".to_string());
        serde_pickle::to_writer(&mut File::create(&file)?, &m1_sms, Default::default())?;
        println!("saved modes into {file}");
    }

    Ok(())
}
