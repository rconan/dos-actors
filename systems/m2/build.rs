fn main() {
    if option_env!("FEM_REPO").is_some() {
        println!("cargo:rustc-cfg=fem");
        let (input_names, _output_names) = gmt_fem_code_builder::io_names(env!("CARGO_PKG_NAME"))
            .expect("failed to get FEM inputs/ouputs names");
        if input_names.find("MCM2S1VCDeltaF").is_some() {
            println!("cargo:warning=gmt_dos-clients_m2-ctrl: ASM inputs detected");
            println!("cargo:rustc-cfg=fem_with_asm")
        }
    }
    println!("cargo:rerun-if-env-changed=FEM_REPO");
    println!("cargo:rerun-if-changed=build.rs");
}
