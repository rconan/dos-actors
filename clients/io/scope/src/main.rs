pub use gmt_dos_clients_io_scope::*;
use std::error::Error;

#[cfg(signal)]
type Signal = <IOScope as IOScopeTrait>::Signal;

#[tokio::main]
async fn main() -> Result<(), Box<dyn Error>> {
    #[cfg(signal)]
    gmt_dos_clients_scope_client::Scope::new().signal::<Signal>()?.show();
    Ok(())
}
