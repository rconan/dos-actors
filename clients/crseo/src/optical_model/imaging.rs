use crseo::Imaging;
use gmt_dos_clients_io::optics::{Dev, Frame, Host};
use interface::{Data, Write};

use super::OpticalModel;

impl Write<Frame<Dev>> for OpticalModel<Imaging> {
    fn write(&mut self) -> Option<Data<Frame<Dev>>> {
        self.select_device();
        self.sensor.as_mut().map(|imgr| {
            let data = imgr.frame().clone();
            Data::new(data)
        })
    }
}

impl Write<Frame<Host>> for OpticalModel<Imaging> {
    fn write(&mut self) -> Option<Data<Frame<Host>>> {
        self.select_device();
        self.sensor
            .as_mut()
            .map(|imgr| { Vec::<f32>::from(&mut imgr.frame()) }.into())
    }
}
