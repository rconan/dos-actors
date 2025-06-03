//
mod controller;
pub use controller::{FsmSegmentInnerController, PiezoStackController};

#[cfg(test)]
mod tests {
    use std::error::Error;

    use gmt_dos_clients_fem::{DiscreteModalSolver, solvers::ExponentialMatrix};
    use gmt_dos_clients_io::{
        gmt_fem::{inputs::MCM2PZTF, outputs::MCM2PZTD},
        gmt_m2::fsm::{
            M2FSMPiezoForces, M2FSMPiezoNodes,
            segment::{FsmCommand, PiezoForces, PiezoNodes},
        },
    };
    use interface::{Read, Update, Write};

    use super::*;

    // cargo t -r --lib -- fsm::tests::controller --exact --nocapture
    #[test]
    pub fn controller() -> Result<(), Box<dyn Error>> {
        const SID: u8 = 7;
        let fem = gmt_fem::FEM::from_env()?;
        let pzt_cmd_p = vec![1e-6, 0.5e-6, -1e-6];

        let mut forces = vec![vec![0f64; 6]; 7];

        let mut plant = DiscreteModalSolver::<ExponentialMatrix>::from_fem(fem)
            .sampling(1e3)
            .proportional_damping(2. / 100.)
            .ins::<MCM2PZTF>()
            .outs::<MCM2PZTD>()
            .use_static_gain_compensation()
            .build()?;

        type CTRLR = FsmSegmentInnerController<SID>;
        let mut ctrlr = CTRLR::new();

        let mut data = vec![];
        let i = (SID as usize - 1) * 6;

        let rss_err = loop {
            let pzt_d =
                <DiscreteModalSolver<_> as Write<M2FSMPiezoNodes>>::write(&mut plant).unwrap();

            let diff_d: Vec<_> = pzt_d
                .chunks(6)
                .nth(SID as usize - 1)
                .unwrap()
                .chunks(2)
                .map(|x| x[1] - x[0])
                .collect();

            let cmd_err: Vec<_> = pzt_cmd_p.iter().zip(&diff_d).map(|(x, y)| x - y).collect();
            <CTRLR as Read<PiezoNodes<SID>>>::read(&mut ctrlr, (&pzt_d[i..i + 6]).into());
            <CTRLR as Read<FsmCommand<SID>>>::read(&mut ctrlr, pzt_cmd_p.clone().into());
            ctrlr.update();
            let seg_forces = <CTRLR as Write<PiezoForces<SID>>>::write(&mut ctrlr).unwrap();
            // dbg!(&seg_forces);
            forces[SID as usize - 1] = seg_forces.as_slice().to_vec();
            <DiscreteModalSolver<_> as Read<M2FSMPiezoForces>>::read(
                &mut plant,
                forces.iter().cloned().flatten().collect::<Vec<_>>().into(),
            );
            plant.update();

            data.push(diff_d);

            let rss_err = 1e6 * (cmd_err.into_iter().map(|x| x * x).sum::<f64>() / 3f64).sqrt();
            if data.len() > 1000 {
                break rss_err;
            }
        };
        assert!(dbg!(rss_err) < 1e-4);
        #[cfg(feature = "complot")]
        {
            let _ = data
                .into_iter()
                .enumerate()
                .map(|(i, data)| {
                    (
                        i as f64 * 1e-3,
                        data.into_iter().map(|x| x * 1e6).collect::<Vec<_>>(),
                    )
                })
                .collect::<complot::Plot>();
        }
        Ok(())
    }
}
