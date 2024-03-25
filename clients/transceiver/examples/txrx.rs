use interface::{Data, Read, UniqueIdentifier, Update, UID};
use log::info;

#[derive(UID)]
#[uid(port = 5001)]
pub enum Sin {}

#[derive(UID)]
#[uid(port = 5002)]
pub enum ISin {}

#[derive(UID)]
#[uid(port = 5003)]
pub enum Blah {}

#[derive(Debug)]
pub struct Print;

impl Update for Print {}

impl<U: UniqueIdentifier<DataType = Vec<f64>>> Read<U> for Print {
    fn read(&mut self, data: Data<U>) {
        info!("{}: {:.3?}", std::any::type_name::<U>(), Vec::from(data));
    }
}
