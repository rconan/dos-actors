use crseo::Imaging;
use gmt_dos_clients_io::optics::{Dev, Frame, Host};
use interface::{Data, Write};

use super::OpticalModel;

impl Write<Frame<Dev>> for OpticalModel<Imaging> {
    fn write(&mut self) -> Option<Data<Frame<Dev>>> {
        self.sensor
            .as_mut()
            .map(|imgr| Data::new(imgr.frame().clone()))
    }
}

impl Write<Frame<Host>> for OpticalModel<Imaging> {
    fn write(&mut self) -> Option<Data<Frame<Host>>> {
        self.sensor
            .as_mut()
            .map(|imgr| { Vec::<f32>::from(&mut imgr.frame()) }.into())
    }
}
