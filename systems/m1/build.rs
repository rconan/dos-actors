fn main() {
    if option_env!("FEM_REPO").is_some() {
        println!("cargo:rustc-cfg=fem");
        gmt_fem_code_builder::rustc_config(env!("CARGO_PKG_NAME"), None).unwrap();
    }
    println!("cargo:rerun-if-env-changed=FEM_REPO");
    println!("cargo:rerun-if-changed=build.rs");
}
