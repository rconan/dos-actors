use gmt_dos_clients_fem::{DiscreteModalSolver, DiscreteStateSpace, ExponentialMatrix};
use gmt_dos_clients_io::gmt_fem::{
    inputs::{MCM2Lcl6F, MCM2SmHexF, OSSM1Lcl6F, CFD2021106F},
    outputs::{
        M2Segment1AxialD, M2Segment2AxialD, M2Segment3AxialD, M2Segment4AxialD, M2Segment5AxialD,
        M2Segment6AxialD, M2Segment7AxialD, MCM2Lcl6D, MCM2SmHexD, OSSM1Lcl, MCM2RB6D,
    },
};
use gmt_fem::FEM;
use matio_rs::MatFile;

fn main() -> anyhow::Result<()> {
    let fem = DiscreteStateSpace::from(FEM::from_env().unwrap())
        .sampling(8e3)
        .use_static_gain_compensation()
        .including_mount()
        .including_m1(None)
        .unwrap()
        .including_asms(Some(vec![1, 2, 3, 4, 5, 6, 7]), None, None)
        .unwrap()
        .ins::<CFD2021106F>()
        .ins::<OSSM1Lcl6F>()
        .ins::<MCM2Lcl6F>()
        .outs::<OSSM1Lcl>()
        .outs::<MCM2Lcl6D>()
        .ins::<MCM2SmHexF>()
        .outs::<MCM2SmHexD>()
        .outs::<MCM2RB6D>()
        .outs::<M2Segment1AxialD>()
        .outs::<M2Segment2AxialD>()
        .outs::<M2Segment3AxialD>()
        .outs::<M2Segment4AxialD>()
        .outs::<M2Segment5AxialD>()
        .outs::<M2Segment6AxialD>()
        .outs::<M2Segment7AxialD>();
    let static_gain = fem.static_gain();
    let dms: DiscreteModalSolver<ExponentialMatrix> = fem.build()?;
    MatFile::save("dc-gain-mismatch.mat")?
        .var("dc_gain", dms.psi_dcg.as_ref().unwrap())?
        .var("static_gain", static_gain.as_ref().unwrap())?;
    Ok(())
}
