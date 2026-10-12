use std::ptr;

use gmt_dos_clients_io::mount::{MountEncoders, MountSetPoint, MountTorques, MountTorquesFeedForward};
use interface::{Data, Read, Size, Update, Write};

use crate::Mount;

impl Size<MountEncoders> for Mount {
    fn len(&self) -> usize {
        14
    }
}
impl Read<MountEncoders> for Mount {
    fn read(&mut self, data: Data<MountEncoders>) {
        unsafe {
            ptr::copy_nonoverlapping(
                (**data).as_ptr(),
                self.control.inputs.Mount_FB.as_mut_ptr(),
                <Mount as Size<MountEncoders>>::len(self),
            )
        }
        unsafe {
            ptr::copy_nonoverlapping(
                (**data).as_ptr(),
                self.drive.inputs.Mount_drv_Po.as_mut_ptr(),
                14,
            )
        }
    }
}
impl Size<MountSetPoint> for Mount {
    fn len(&self) -> usize {
        3
    }
}
impl Read<MountSetPoint> for Mount {
    fn read(&mut self, data: Data<MountSetPoint>) {
        unsafe {
            ptr::copy_nonoverlapping(
                (**data).as_ptr(),
                self.control.inputs.Mount_SP.as_mut_ptr(),
                <Mount as Size<MountSetPoint>>::len(self),
            )
        }
    }
}
impl Update for Mount {
    fn update(&mut self) {
        self.control.step();
        unsafe {
            ptr::copy_nonoverlapping(
                self.control.outputs.Mount_cmd.as_ptr(),
                self.drive.inputs.Mount_cmd.as_mut_ptr(),
                3,
            )
        }
        self.drive.step();
    }
}
#[cfg(mount = "FDR")]
impl Size<MountTorques> for Mount {
    fn len(&self) -> usize {
        16
    }
}
#[cfg(mount = "PDR")]
impl Size<MountTorques> for Mount {
    fn len(&self) -> usize {
        20
    }
}

impl Write<MountTorques> for Mount {
    fn write(&mut self) -> Option<Data<MountTorques>> {
        let mut data = vec![0f64; <Mount as Size<MountTorques>>::len(self)];
        unsafe {
            ptr::copy_nonoverlapping(
                self.drive.outputs.Mount_T.as_ptr(),
                data.as_mut_ptr(),
                data.len(),
            )
        };
        // feed-forward added after the drives saturation
        data.iter_mut()
            .zip(&self.torques_feedforward)
            .for_each(|(t, ff)| *t += ff);
        Some(data.into())
    }
}

/// Drive torques feed-forward, added to the drive torques after the drives saturation
impl Read<MountTorquesFeedForward> for Mount {
    fn read(&mut self, data: Data<MountTorquesFeedForward>) {
        self.torques_feedforward = data.into_arc().to_vec();
    }
}

/* #[cfg(gir_tooth)]
impl Write<gmt_dos_clients_io::gmt_fem::inputs::OSSGIRTooth6F> for Mount {
    fn write(&mut self) -> Option<Data<gmt_dos_clients_io::gmt_fem::inputs::OSSGIRTooth6F>> {
        let data = vec![self.drive.outputs.ToothCAxialFo];
        Some(data.into())
    }
} */
