pub use gmt_dos_clients_io_scope::*;
use gmt_dos_clients_scope_client::Scope;
use std::error::Error;

type Signal = <IOScope as IOScopeTrait>::Signal;

#[tokio::main]
async fn main() -> Result<(), Box<dyn Error>> {
    Scope::new().signal::<Signal>()?.show();
    Ok(())
}
