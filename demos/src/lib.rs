use std::sync::Arc;

use gmt_dos_clients_io::optics::WfeRms;
use gmt_dos_clients_lom::LinearOpticalModel;
use interface::{Data, Read, UID, UniqueIdentifier, Update, Write};

#[derive(UID)]
#[alias(name = WfeRms<-6>, port = 55991, client = LinearOpticalModel, traits = Write)]
pub enum M1RbmWfeRms {}

#[derive(UID)]
#[alias(name = WfeRms<-6>, port = 55992, client = LinearOpticalModel, traits = Write)]
pub enum AsmShellWfeRms {}

#[derive(UID)]
#[alias(name = WfeRms<-6>, port = 55993, client = LinearOpticalModel, traits = Write)]
pub enum AsmRefBodyWfeRms {}

#[derive(UID)]
pub enum RBMCmd {}
#[derive(UID)]
pub enum ActuatorCmd {}
#[derive(UID)]
pub enum ASMSCmd {}

#[derive(Debug, Default)]
pub struct Multiplex {
    data: Arc<Vec<f64>>,
    slices: Vec<usize>,
}
impl Multiplex {
    pub fn new(slices: Vec<usize>) -> Self {
        Self {
            slices,
            ..Default::default()
        }
    }
}

impl Update for Multiplex {}
impl<U: UniqueIdentifier<DataType = Vec<f64>>> Read<U> for Multiplex {
    fn read(&mut self, data: Data<U>) {
        self.data = data.into_arc();
    }
}
impl<U: UniqueIdentifier<DataType = Vec<Arc<Vec<f64>>>>> Write<U> for Multiplex {
    fn write(&mut self) -> Option<Data<U>> {
        let mut mx_data = vec![];
        let data = self.data.as_slice();
        let mut a = 0_usize;
        for s in &self.slices {
            let b = a + *s;
            mx_data.push(Arc::new(data[a..b].to_vec()));
            a = b;
        }
        Some(mx_data.into())
    }
}
