use gmt_dos_clients_io::optics::{Dev, Frame, Host};
use interface::{Data, Write};

use crate::OpticalModel;

use super::Camera;

impl<const I: usize> Write<Frame<Dev>> for OpticalModel<Camera<I>> {
    fn write(&mut self) -> Option<Data<Frame<Dev>>> {
        self.sensor.as_mut().map(|imgr| {
            let frame = imgr.frame().clone();
            imgr.reset();
            Data::new(frame)
        })
    }
}

impl<const I: usize> Write<Frame<Host>> for OpticalModel<Camera<I>> {
    fn write(&mut self) -> Option<Data<Frame<Host>>> {
        self.sensor.as_mut().map(|imgr| {
            {
                let frame = Vec::<f32>::from(&mut imgr.frame());
                imgr.reset();
                frame
            }
            .into()
        })
    }
}
