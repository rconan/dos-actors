use std::{
    env::{self, VarError},
    error::Error,
    fmt::Display,
    path::Path,
};

#[derive(Debug)]
pub struct CudaCompilerPathError(VarError, VarError);
impl Error for CudaCompilerPathError {}
impl Display for CudaCompilerPathError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let Self(e1, e2) = self;
        e1.fmt(f)?;
        e2.fmt(f)
    }
}

fn main() -> Result<(), Box<dyn Error>> {
    match (env::var("NVCC"), env::var("CUDACXX")) {
        (Ok(_), Ok(_)) => (),
        (Ok(var), Err(_)) => unsafe {
            env::set_var("CUDACXX", var);
        },
        (Err(_), Ok(var)) => unsafe {
            env::set_var("NVCC", var);
        },
        (Err(e1), Err(e2)) => {
            println!("cargo::error=neither NVCC nor CUDACXX environment variables are set");
            return Err(Box::new(CudaCompilerPathError(e1, e2)));
        }
    };
    cc::Build::new()
        .cuda(true)
        .file("src/solver.cu")
        .compile("femcudasolver");
    println!("cargo:rustc-link-lib=cublas");
    bindgen::builder()
        .header("src/solver.hpp")
        .wrap_unsafe_ops(true)
        .clang_arg("-I/usr/local/cuda/include")
        .allowlist_type("mode_state_space")
        .allowlist_type("state_space")
        .generate()?
        .write_to_file(Path::new(&env::var("OUT_DIR")?).join("bindings.rs"))?;
    println!("cargo::rerun-if-changed=src/solver.cu");
    println!("cargo::rerun-if-changed=src/solver.hpp");
    Ok(())
}
