//! # GIF image encoder
//!
//! A client to create a GIF image from a stream of frame

use std::{
    fmt::Debug,
    fs::File,
    ops::{Div, Sub},
    path::Path,
    sync::Arc,
};

use colorous::CIVIDIS;
use gif::{Encoder, EncodingError, Frame, Repeat};
use interface::{Read, UniqueIdentifier, Update};

pub struct Gif<T> {
    frame: Arc<Vec<T>>,
    width: usize,
    height: usize,
    delay: u16,
    encoder: Encoder<File>,
}

#[derive(Debug, thiserror::Error)]
pub enum GifError {
    #[error("gif error: {0}")]
    Gif(#[from] EncodingError),
    #[error("io error: {0}")]
    Io(#[from] std::io::Error),
}

type Result<T> = std::result::Result<T, GifError>;

impl<T> Gif<T> {
    /// Creates a new GIF encoder
    ///
    /// The `width` and `height` of the image must match the frame size
    pub fn new<P: AsRef<Path>>(path: P, width: usize, height: usize) -> Result<Self> {
        let file = File::create(path.as_ref())?;
        let mut encoder = Encoder::new(file, width as u16, height as u16, &[])?;
        encoder.set_repeat(Repeat::Infinite)?;
        Ok(Self {
            frame: Default::default(),
            width,
            height,
            delay: 10,
            encoder,
        })
    }
    /// Frame delay in milliseconds
    pub fn delay(mut self, delay: usize) -> Self {
        self.delay = delay as u16 / 10;
        self
    }
}

impl<T> Update for Gif<T>
where
    T: Send + Sync + PartialOrd + Div<Output = T> + Debug + Copy + Sub<Output = T>,
    f64: From<T>,
{
    fn update(&mut self) {
        let max_px = *self
            .frame
            .iter()
            .max_by(|&a, &b| a.partial_cmp(b).unwrap())
            .unwrap();
        let min_px = *self
            .frame
            .iter()
            .min_by(|&a, &b| a.partial_cmp(b).unwrap())
            .unwrap();
        let colormap = CIVIDIS;
        let pixels: Vec<_> = self
            .frame
            .iter()
            .map(|x| (*x - min_px) / (max_px - min_px))
            .map(|t| colormap.eval_continuous(t.into()))
            .flat_map(|c| c.into_array().to_vec())
            .collect();
        let mut frame = Frame::from_rgb(self.width as u16, self.height as u16, &pixels);
        frame.delay = self.delay;
        self.encoder
            .write_frame(&frame)
            .expect("failed to write frame to GIF encoder");
    }
}

impl<T, U> Read<U> for Gif<T>
where
    T: Send + Sync + PartialOrd + Div<Output = T> + Debug + Copy + Sub<Output = T>,
    f64: From<T>,
    U: UniqueIdentifier<DataType = Vec<T>>,
{
    fn read(&mut self, data: interface::Data<U>) {
        self.frame = data.into_arc();
    }
}
