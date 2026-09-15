use std::{env, error::Error, fs, path::Path};

fn main() -> Result<(), Box<dyn Error>> {
    println!("cargo::rustc-check-cfg=cfg(signal)");

    let scope = match env::var("SIGNAL") {
        Ok(signal) => {
            println!("cargo::rustc-cfg=signal");
            if let Some(u) = env::var("U").ok() {
                format!(
                    r##"
impl IOScopeTrait for IOScope {{
    type Signal = interface::units::{}<gmt_dos_clients_io::{}>;
}}
"##,
                    u, signal
                )
            } else {
                format!(
                    r##"
impl IOScopeTrait for IOScope {{
    type Signal = gmt_dos_clients_io::{};
}}
"##,
                    signal
                )
            }
        }
        Err(e) => {
            println!("cargo::warning=SIGNAL missing due to {e}, aborting scope");
            return Ok(())
        }
    };

    let out_dir = env::var("OUT_DIR")?;
    let out_file = Path::new(&out_dir).join("scope.rs");
    fs::write(&out_file, scope)?;

    println!("cargo::rerun-if-env-changed=SIGNAL");
    Ok(())
}
