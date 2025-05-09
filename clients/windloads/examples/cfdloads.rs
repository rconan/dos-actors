use gmt_dos_clients_windloads::{CfdLoads, WindLoads, WindLoadsBuilder};
use parse_monitors::MonitorsLoader;

fn main() -> anyhow::Result<()> {
    env_logger::init();
    let mut loader = MonitorsLoader::<2021>::default();
    let mut monitors = loader.load()?;
    let keys: Vec<_> = monitors.forces_and_moments.into_keys().collect();
    println!("{keys:?}");

    let mut fem = gmt_fem::FEM::from_env()?;

    let cfd_loads_client = CfdLoads::foh(".", 1_000)
        // .duration(sim_duration as f64)
        // .mount(&mut fem, 0, None)
        // .m1_segments(&mut fem, 0)
        // .m2_segments(&mut fem, 0)
        .windloads(
            &mut fem,
            0,
            // WindLoadsBuilder::new().mount(Some(vec![
            //     WindLoads::TopEnd,
            //     WindLoads::M2Baffle,
            //     WindLoads::M1Baffle,
            //     WindLoads::Trusses,
            //     WindLoads::PrimeFocusArms, // WindLoads::MirrorCovers,
            //                                // WindLoads::LaserGuideStars,
            //                                // WindLoads::CRings,
            //                                // WindLoads::GIR,
            //                                // WindLoads::Platforms,
            // ])),
            Default::default(),
        )
        .build()?;

    Ok(())
}
