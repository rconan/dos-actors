//! # GIF image encoder
//!
//! A client to create a GIF image from a stream of frame

use std::{
    fmt,
    fs::File,
    ops::{Div, Sub},
    path::Path,
};

use gif::{Encoder, EncodingError, Frame as GifFrame};
use interface::{Read, UniqueIdentifier, Update};

mod frame;
pub use frame::Frame;

pub struct Gif<T> {
    width: usize,
    height: usize,
    delay: u16,
    encoder: Encoder<File>,
    frame: Frame<T>,
}

#[derive(Debug, thiserror::Error)]
pub enum GifError {
    #[error("gif error: {0}")]
    Gif(#[from] EncodingError),
    #[error("io error: {0}")]
    Io(#[from] std::io::Error),
    #[error("image error")]
    Image(#[from] image::error::ImageError),
}

type Result<T> = std::result::Result<T, GifError>;

pub trait FrameBuilder {
    /// Sets the size of the image
    ///
    /// Per default the image size is set to the size of the data
    fn image_size(self, n: usize) -> Result<Self>
    where
        Self: Sized;
    fn font_scale(self, s: f32) -> Self;
}
impl<T> FrameBuilder for Gif<T> {
    fn image_size(mut self, n: usize) -> Result<Self> {
        self.frame = <_ as FrameBuilder>::image_size(self.frame, n)?;
        if let Some((_, height)) = self.frame.get_image_size() {
            self.width = height * self.width / self.height;
            self.height = height;
            let file = File::create(self.frame.path())?;
            self.encoder = Encoder::new(file, self.width as u16, self.height as u16, &[])?;
        }
        Ok(self)
    }
    fn font_scale(mut self, s: f32) -> Self {
        self.frame = <_ as FrameBuilder>::font_scale(self.frame, s);
        self
    }
}

impl<T> Gif<T> {
    /// Creates a new GIF encoder
    ///
    /// The `width` and `height` of the image must match the frame size
    pub fn new<P: AsRef<Path>>(path: P, width: usize, height: usize) -> Result<Self> {
        let frame = Frame::<T>::new(path, height).autosave(false);
        let file = File::create(frame.path())?;
        let encoder = Encoder::new(file, width as u16, height as u16, &[])?;
        // encoder.set_repeat(Repeat::Infinite)?;
        Ok(Self {
            // path,
            frame,
            width,
            height,
            delay: 100,
            encoder,
            // idx: 0,
            // font: font.into(),
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
    T: Send
        + Sync
        + PartialOrd
        + Div<Output = T>
        + fmt::Debug
        + Copy
        + Sub<Output = T>
        + fmt::LowerExp,
    f64: From<T>,
{
    fn update(&mut self) {
        <Frame<T> as Update>::update(&mut self.frame);

        let mut q = self.frame.image().clone().into_raw();
        let mut frame =
            GifFrame::from_rgba_speed(self.width as u16, self.height as u16, &mut q, 10);
        frame.delay = self.delay;
        self.encoder
            .write_frame(&frame)
            .expect("failed to write frame to GIF encoder");
    }
}
// Helper function to draw semi-transparent guide lines
/* fn draw_guide_lines(image: &mut RgbaImage, width: u32, height: u32) {
    // Semi-transparent gray color (RGB: 128,128,128, Alpha: 128)
    let line_color = Rgba([128u8, 128u8, 128u8, 128u8]);

    // Draw horizontal line
    for x in 0..width {
        let y = height / 2;
        image.put_pixel(x, y, line_color);
        // Make the line 3 pixels thick
        image.put_pixel(x, y - 1, line_color);
        if (height & 1) == 1 {
            image.put_pixel(x, y + 1, line_color);
        }
    }

    // Draw vertical line
    for y in 0..height {
        let x = width / 2;
        image.put_pixel(x, y, line_color);
        // Make the line 3 pixels thick
        image.put_pixel(x - 1, y, line_color);
        if (width & 1) == 1 {
            image.put_pixel(x + 1, y, line_color);
        }
    }
} */

impl<T, U> Read<U> for Gif<T>
where
    T: Send
        + Sync
        + PartialOrd
        + Div<Output = T>
        + fmt::Debug
        + Copy
        + Sub<Output = T>
        + fmt::LowerExp,
    f64: From<T>,
    U: UniqueIdentifier<DataType = Vec<T>>,
    Vec<T>: From<interface::Data<U>>,
{
    fn read(&mut self, data: interface::Data<U>) {
        <Frame<T> as Read<U>>::read(&mut self.frame, data);
    }
}
