use std::{env, fmt::Display, fs, path::Path};

pub struct Assembly(Vec<u8>);

impl Default for Assembly {
    fn default() -> Self {
        Self(vec![1, 2, 3, 4, 5, 6, 7])
    }
}

impl From<String> for Assembly {
    fn from(sids: String) -> Self {
        Self({
            let mut sids: Vec<_> = sids
                .split(',')
                .map(|sid| {
                    let sid = sid.trim();
                    let id = sid.parse::<u8>().unwrap_or_else(|_| {
                        panic!("ASSEMBLY: expected a segment id, found {sid:?}")
                    });
                    assert!(
                        (1..=7).contains(&id),
                        "ASSEMBLY: expected segment ids in [1,7], found {id}"
                    );
                    id
                })
                .collect();
            sids.sort();
            sids.dedup();
            sids
        })
    }
}

impl Display for Assembly {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let n = self.0.len();
        write!(
            f,
            r#"
        /// GMT assembly definition
        pub trait Assembly {{
            const N: usize = {0};
            const SIDS: [u8; {0}] = {1:?};
        
            fn position<const ID: u8>() -> Option<usize> {{
                <Self as Assembly>::SIDS
                    .into_iter()
                    .position(|sid| sid == ID)
            }}
        }}"#,
            n, self.0
        )
    }
}

fn main() -> anyhow::Result<()> {
    gmt_fem_code_builder::generate_io(env!("CARGO_PKG_NAME"))?;

    let assembly: Assembly = env::var("ASSEMBLY").map_or(Default::default(), |e| e.into());

    let out_dir = env::var_os("OUT_DIR").unwrap();
    let dest_path = Path::new(&out_dir).join("assembly.rs");
    fs::write(dest_path, assembly.to_string())?;

    println!("cargo:rerun-if-env-changed=ASSEMBLY");

    Ok(())
}
