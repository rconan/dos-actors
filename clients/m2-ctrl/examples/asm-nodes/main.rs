use gmt_dos_clients_fem::fem_io;
use gmt_dos_clients_m2_ctrl::nodes;
use gmt_fem::FEM;

fn main() -> anyhow::Result<()> {
    let fem = FEM::from_env()?;
    println!("{fem}");

    for i in 1..=7 {
        let mut nodes = nodes::AsmsNodes::default();
        // let io_name = format!("M2_segment_{i}_axial_d");
        let io_name = format!("M1_actuators_segment_{i}");
        // let io_name = format!("M1_segment_{i}_axial_d");
        println!("Loading nodes from {io_name}");
        let xyz = match (
            Box::<dyn fem_io::GetIn>::try_from(io_name.clone()),
            Box::<dyn fem_io::GetOut>::try_from(io_name.clone()),
        ) {
            (Ok(_), Ok(_)) => unimplemented!(),
            (Ok(get_in), Err(_)) => {
                let idx = get_in.position(&fem.inputs).expect(&format!(
                    "failed to find the index of the output: {io_name}"
                ));
                fem.inputs[idx]
                    .as_ref()
                    .map(|i| i.get_by(|i| i.properties.location.clone()))
                    .expect(&format!("failed to read nodes locations from {io_name}"))
            }
            (Err(_), Ok(get_out)) => {
                let idx = get_out.position(&fem.outputs).expect(&format!(
                    "failed to find the index of the output: {io_name}"
                ));
                fem.outputs[idx]
                    .as_ref()
                    .map(|i| i.get_by(|i| i.properties.location.clone()))
                    .expect(&format!("failed to read nodes locations from {io_name}"))
            }
            (Err(_), Err(_)) => panic!("failed to locate {} in {}", io_name, env!("FEM_REPO")),
        };
        nodes.push(nodes::Nodes { sid: i as u8, xyz });

        // nodes.into_bin("ASMS-nodes.bin")?;
        // nodes.into_parquet("ASMS-nodes.parquet")?;
        nodes.into_parquet(format!("m1s{i}_actuators-nodes.parquet"))?;
    }

    Ok(())
}
