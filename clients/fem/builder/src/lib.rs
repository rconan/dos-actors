use std::{
    env,
    fs::{self, File},
    io::Read,
    path::Path,
};

use apache_arrow::{
    self as arrow,
    array::{LargeStringArray, StringArray},
    record_batch::RecordBatchReader,
};
use bytes::Bytes;
use parquet::arrow::arrow_reader::ParquetRecordBatchReaderBuilder;
use zip::ZipArchive;

#[derive(thiserror::Error, Debug)]
pub enum Error {
    #[error("No suitable record in file")]
    NoRecord,
    #[error("No suitable data in file")]
    NoData,
    #[error("Cannot read arrow table")]
    ReadArrow(#[from] arrow::error::ArrowError),
    #[error("Cannot read parquet file")]
    ReadParquet(#[from] parquet::errors::ParquetError),
    #[error("Cannot find archive in zip file")]
    Zip(#[from] zip::result::ZipError),
    #[error("Cannot read zip file content")]
    ReadZip(#[from] std::io::Error),
}

mod names;
pub use names::{Name, Names};
mod io;
pub use io::IO;
mod get_io;
pub use get_io::GetIO;

use apache_arrow::datatypes::Schema;
use apache_arrow::record_batch::RecordBatch;
use std::sync::Arc;
fn get_data(
    field: &str,
    fem_io: &str,
    schema: Arc<Schema>,
    table: &RecordBatch,
) -> Option<Vec<String>> {
    let (idx, _) = schema.column_with_name(field).expect(&format!(
        r#"failed to get {}puts "{}" index with field:\n{:}"#,
        fem_io,
        field,
        schema.field_with_name(field).unwrap()
    ));
    match schema.field_with_name(field).unwrap().data_type() {
        arrow::datatypes::DataType::Utf8 => table
            .column(idx)
            .as_any()
            .downcast_ref::<StringArray>()
            .expect(&format!(
                r#"failed to get {}puts "group" data at index #{} from field\n{:}"#,
                fem_io,
                idx,
                schema.field_with_name("group").unwrap()
            ))
            .iter()
            .map(|x| x.map(|x| x.to_owned()))
            .collect(),
        arrow::datatypes::DataType::LargeUtf8 => table
            .column(idx)
            .as_any()
            .downcast_ref::<LargeStringArray>()
            .expect(&format!(
                r#"failed to get {}puts "group" data at index #{} from field\n{:}"#,
                fem_io,
                idx,
                schema.field_with_name("group").unwrap()
            ))
            .iter()
            .map(|x| x.map(|x| x.to_owned()))
            .collect(),
        other => panic!(
            r#"Expected "Uft8" or "LargeUtf8" datatype, found {}"#,
            other
        ),
    }
}

// Read the fields
fn get_fem_io(zip_file: &mut ZipArchive<File>, fem_io: &str) -> Result<Names, Error> {
    println!("FEM_{}PUTS", fem_io.to_uppercase());
    let Ok(mut input_file) = zip_file.by_name(&format!(
        "rust/modal_state_space_model_2ndOrder_{}.parquet",
        fem_io
    )) else {
        panic!(
            r#"cannot find "rust/modal_state_space_model_2ndOrder_{}.parquet" in archive"#,
            fem_io
        )
    };
    let mut contents: Vec<u8> = Vec::new();
    input_file.read_to_end(&mut contents)?;

    let Ok(parquet_reader) = ParquetRecordBatchReaderBuilder::try_new(Bytes::from(contents)) else {
        panic!("failed to create `ParquetRecordBatchReaderBuilder`")
    };
    let Ok(parquet_reader) = parquet_reader.with_batch_size(2048).build() else {
        panic!("failed to create `ParquetRecordBatchReader`")
    };
    let schema = parquet_reader.schema();

    parquet_reader
        .map(|maybe_table| {
            if let Ok(table) = maybe_table {
                get_data("group", fem_io, schema.clone(), &table)
                    .zip(get_data("description", fem_io, schema.clone(), &table))
                    .ok_or(Error::NoData)
            } else {
                Err(Error::NoRecord)
            }
        })
        .collect::<Result<Vec<_>, Error>>()
        .map(|data| {
            let (n, d): (Vec<_>, Vec<_>) = data.into_iter().unzip();
            let n: Vec<_> = n.into_iter().flatten().collect();
            let d: Vec<_> = d.into_iter().flatten().collect();
            (n, d)
        })
        // .map(|data| data.into_iter().flatten().collect::<Vec<_>>())
        .map(|data| {
            let (name, description) = data;
            let mut data_iter = name.into_iter();
            let mut description_iter = description.into_iter();
            let mut name = data_iter.next().unwrap();
            let mut names: Vec<Name> = vec![Name::from(&name)];
            names
                .last_mut()
                .map(|name| name.push_description(description_iter.next().unwrap()));
            loop {
                match data_iter.next() {
                    Some(data) if data == name => {
                        names
                            .last_mut()
                            .map(|name| name.push_description(description_iter.next().unwrap()));
                    }
                    Some(data) => {
                        name = data;
                        names.push(name.as_str().into());
                        names
                            .last_mut()
                            .map(|name| name.push_description(description_iter.next().unwrap()));
                    }
                    None => break,
                }
            }
            names.into_iter().collect()
        })
}

pub fn io_names(from_crate: &str) -> std::result::Result<(Names, Names), Error> {
    Ok(if let Ok(fem_repo) = env::var("FEM_REPO") {
        // Gets the FEM repository
        println!(
        "cargo:warning={}: generating FEM/Actors interface code based on the FEM inputs and outputs tables in {}",
        from_crate,fem_repo
    );
        // Opens the mat file
        let path = Path::new(&fem_repo);
        let Ok(file) = File::open(path.join("modal_state_space_model_2ndOrder.zip")) else {
            panic!("Cannot find `modal_state_space_model_2ndOrder.zip` in `FEM_REPO`");
        };
        let mut zip_file = zip::ZipArchive::new(file)?;

        let Ok(input_names) = get_fem_io(&mut zip_file, "in") else {
            panic!("failed to parse FEM inputs variables")
        };
        let Ok(output_names) = get_fem_io(&mut zip_file, "out") else {
            panic!("failed to parse FEM outputs variables")
        };
        (input_names, output_names)
    } else {
        println!("cargo:warning=the FEM_REPO environment variable is not set, using dummy inputs and outputs instead");
        let (inputs, outputs): (Vec<_>, Vec<_>) = (1..=5)
            .map(|i| {
                (
                    String::from(format!("In{i}")),
                    String::from(format!("Out{i}")),
                )
            })
            .unzip();
        (inputs.into_iter().collect(), outputs.into_iter().collect())
    })
}

/// Generate the code for interfacing [dos-actors] to [gmt-fem]
pub fn generate_interface(from_crate: &str) -> anyhow::Result<()> {
    let (input_names, output_names): (Names, Names) = io_names(from_crate)?;

    let out_dir = env::var_os("OUT_DIR").unwrap();
    let dest_path = Path::new(&out_dir);

    fs::write(
        dest_path.join("fem_get_in.rs"),
        format!("{}", GetIO::new("In", &input_names)),
    )?;
    fs::write(
        dest_path.join("fem_get_out.rs"),
        format!("{}", GetIO::new("Out", &output_names)),
    )?;

    fs::write(
        dest_path.join("fem_inputs.rs"),
        input_names
            .iter()
            .map(|name| format!("{}", name.impl_enum_variant_for_io("Inputs")))
            .collect::<Vec<String>>()
            .join("\n"),
    )?;
    fs::write(
        dest_path.join("fem_outputs.rs"),
        output_names
            .iter()
            .map(|name| format!("{}", name.impl_enum_variant_for_io("Outputs")))
            .collect::<Vec<String>>()
            .join("\n"),
    )?;

    rustc_config(from_crate, Some((input_names, output_names)))?;

    println!("cargo:rerun-if-env-changed=FEM_REPO");
    Ok(())
}

/// Generate the list of inputs and outputs of the FEM dos-actors
pub fn generate_io(from_crate: &str) -> anyhow::Result<()> {
    let (input_names, output_names): (Names, Names) = io_names(from_crate)?;

    let out_dir = env::var_os("OUT_DIR").unwrap();
    let dest_path = Path::new(&out_dir);

    fs::write(
        dest_path.join("fem_actors_inputs.rs"),
        format!("{}", input_names),
    )?;
    fs::write(
        dest_path.join("fem_actors_outputs.rs"),
        format!("{}", output_names),
    )?;

    println!("cargo:rerun-if-env-changed=FEM_REPO");
    Ok(())
}

pub fn rustc_config(from_crate: &str, io: Option<(Names, Names)>) -> anyhow::Result<()> {
    if option_env!("FEM_REPO").is_some() {
        println!("cargo:rustc-cfg=fem");
        let (input_names, output_names): (Names, Names) = match io {
            Some(io) => Ok(io),
            None => io_names(from_crate),
        }?;
        if input_names.find("MCM2S1VCDeltaF").is_some() {
            println!("cargo:warning={}: ASM top-end", from_crate);
            println!(r#"cargo:rustc-cfg=topend="ASM""#)
        }
        if input_names.find("MCM2PZTF").is_some() {
            println!("cargo:warning={}: FSM top-end", from_crate);
            println!(r#"cargo:rustc-cfg=topend="FSM""#);
        }
        match (
            input_names.find("MCM2S1VCDeltaF"),
            input_names.find("MCM2PZTF"),
            input_names.find("MCM2SmHexF"),
        ) {
            (Some(_), None, Some(_)) => {
                println!("cargo:warning={}: ASMS inputs", from_crate);
                println!(r#"cargo:rustc-cfg=m2"#)
            }
            (None, Some(_), Some(_)) => {
                println!("cargo:warning={}: FSMS inputs", from_crate);
                println!(r#"cargo:rustc-cfg=m2"#)
            }
            _ => (),
        };
        if input_names.find("CFD2021106F").is_some() {
            println!("cargo:warning={}: CFD inputs", from_crate);
            println!(r#"cargo:rustc-cfg=cfd2021"#)
        }
        if input_names.find("CFD2025046F").is_some() {
            println!("cargo:warning={}: CFD inputs", from_crate);
            println!(r#"cargo:rustc-cfg=cfd2025"#)
        }
        match (
            input_names.find("OSSAzDriveTorque"),
            input_names.find("OSSElDriveTorque"),
            input_names.find("OSSRotDriveTorque"),
            output_names.find("OSSAzEncoderAngle"),
            output_names.find("OSSElEncoderAngle"),
            output_names.find("OSSRotEncoderAngle"),
        ) {
            (Some(_), Some(_), Some(_), Some(_), Some(_), Some(_)) => {
                println!("cargo:warning={}: Mount inputs and outputs", from_crate);
                println!(r#"cargo:rustc-cfg=mount"#)
            }
            _ => (),
        };
        match (
            input_names.find("OSSHarpointDeltaF"),
            input_names.find("M1ActuatorsSegment1"),
            output_names.find("OSSHardpointD"),
            output_names.find("OSSM1Lcl"),
            output_names.find("M1Segment1AxialD"),
        ) {
            (Some(_), Some(_), Some(_), Some(_), Some(_)) => {
                println!("cargo:warning={}: M1 inputs and outputs", from_crate);
                println!(r#"cargo:rustc-cfg=m1"#)
            }
            _ => (),
        };
        if input_names.find("OSS00GroundAcc").is_some() {
            println!("cargo:warning={}: OSS00GroundAcc input", from_crate);
            println!(r#"cargo:rustc-cfg=ground_acceleration"#)
        }
        if output_names.find("MCM2Lcl6D").is_some() {
            println!("cargo:warning={}: MCM2Lcl6D as M2 RBM output", from_crate);
            println!(r#"cargo:rustc-cfg=m2_rbm="MCM2Lcl6D""#)
        }
        if output_names.find("MCM2Lcl").is_some() {
            println!("cargo:warning={}: MCM2Lcl as M2 RBM output", from_crate);
            println!(r#"cargo:rustc-cfg=m2_rbm="MCM2Lcl""#)
        }
    }
    Ok(())
}

/// Generate the code for loading and for interacting with FEM data
pub fn generate_fem(from_crate: &str) -> anyhow::Result<()> {
    let (input_names, output_names): (Names, Names) = io_names(from_crate)?;

    let out_dir = env::var_os("OUT_DIR").unwrap();
    let dest_path = Path::new(&out_dir);

    fs::write(
        dest_path.join("fem_inputs.rs"),
        format!("{}", IO::new("Inputs", &input_names)),
    )?;
    fs::write(
        dest_path.join("fem_outputs.rs"),
        format!("{}", IO::new("Outputs", &output_names)),
    )?;

    println!("cargo:rerun-if-env-changed=FEM_REPO");

    Ok(())
}
