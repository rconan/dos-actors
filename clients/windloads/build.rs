fn main() {
    gmt_fem_code_builder::rustc_config(env!("CARGO_PKG_NAME"), None).unwrap();
    println!("cargo:rerun-if-env-changed=FEM_REPO");
    println!("cargo:rerun-if-changed=build.rs");
}
