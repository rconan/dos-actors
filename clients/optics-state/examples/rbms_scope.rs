use std::{iter, sync::mpsc::channel, thread, time::Duration};

use gmt_dos_clients_io::gmt_m2::M2RigidBodyMotions;
use tokio::sync::broadcast;

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    let legends: Vec<_> = (1..=7)
        .flat_map(|i| {
            ["Tx", "Ty", "Tz", "Rx", "Ry", "Rz"]
                .into_iter()
                .map(|rbm| format!("M2S{i} {rbm}"))
                .collect::<Vec<_>>()
        })
        .collect();
    // dbg!(&legends);
    let (tx, rx) = broadcast::channel::<Vec<String>>(8);
    let h = thread::spawn(move || {
        thread::sleep(Duration::from_secs(3));
        let hiddens: Vec<_> = (2..=7)
            .flat_map(|i| {
                ["Tx", "Ty", "Tz", "Rx", "Ry", "Rz"]
                    .into_iter()
                    .map(|rbm| format!("M2S{i} {rbm}"))
                    .collect::<Vec<_>>()
            })
            .collect();
        tx.send(hiddens)
    });
    gmt_dos_clients_scope::client::Scope::new()
        .rx(rx)
        .signal_with_legends::<M2RigidBodyMotions>(legends)?
        .show();

    h.join();
    Ok(())
}
