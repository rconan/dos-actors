use gmt_dos_clients_io::optics::{Dev, Frame, Host};
use interface::{Data, Size, Write};

use crate::OpticalModel;

use super::Camera;

impl<const I: usize> Write<Frame<Dev>> for OpticalModel<Camera<I>> {
    fn write(&mut self) -> Option<Data<Frame<Dev>>> {
        self.sensor.as_mut().map(|imgr| {
            let frame = imgr.frame().clone();
            Data::new(frame)
        })
    }
}

impl<const I: usize> Write<Frame<Host>> for OpticalModel<Camera<I>> {
    fn write(&mut self) -> Option<Data<Frame<Host>>> {
        self.sensor.as_mut().map(|imgr| {
            {
                let frame = Vec::<f32>::from(&mut imgr.frame());
                frame
            }
            .into()
        })
    }
}

impl<const I: usize> Size<Frame<Host>> for OpticalModel<Camera<I>> {
    fn len(&self) -> usize {
        self.sensor
            .as_ref()
            .map(|imgr| (imgr.resolution().pow(2) * imgr.n_guide_star()) as usize)
            .unwrap_or_default()
    }
}
