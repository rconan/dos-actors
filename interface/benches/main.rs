use std::sync::Arc;

use criterion::{Criterion, criterion_group, criterion_main};
use gmt_dos_actors_clients_interface::{Data, UniqueIdentifier};

const N: usize = 100_000;

pub enum U {}
impl UniqueIdentifier for U {
    type DataType = Vec<f64>;
}

#[derive(Default)]
pub struct Client {
    pub data: Arc<Vec<f64>>,
}
impl Client {
    pub fn into_arc(&mut self, data: Data<U>) {
        self.data = data.into_arc();
    }
    pub fn as_arc(&mut self, data: &Data<U>) {
        self.data = data.as_arc();
    }
}

pub fn move_arc(c: &mut Criterion) {
    let mut client = Client::default();
    let data = Data::new(vec![0f64; N]);
    c.bench_function("MoveArc", |b| {
        b.iter(|| client.into_arc(<Data<U> as Clone>::clone(&data)))
    });
}

pub fn as_arc(c: &mut Criterion) {
    let mut client = Client::default();
    let data = Data::new(vec![0f64; N]);
    c.bench_function("AsArc", |b| b.iter(|| client.as_arc(&data)));
}

criterion_group!(benches, move_arc, as_arc);
criterion_main!(benches);
