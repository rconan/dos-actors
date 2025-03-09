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

use ab_glyph::{FontArc, FontRef, PxScale};
use colorous::CIVIDIS;
use gif::{Encoder, EncodingError, Frame as GifFrame, Repeat};
use image::{Rgba, RgbaImage};
use imageproc::drawing::draw_text_mut;
use interface::{Read, UniqueIdentifier, Update};

mod frame;
pub use frame::Frame;

pub struct Gif<T> {
    frame: Arc<Vec<T>>,
    width: usize,
    height: usize,
    delay: u16,
    encoder: Encoder<File>,
    idx: usize,
    font: FontArc,
    font_scale: PxScale,
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

impl<T> Gif<T> {
    /// Creates a new GIF encoder
    ///
    /// The `width` and `height` of the image must match the frame size
    pub fn new<P: AsRef<Path>>(path: P, width: usize, height: usize) -> Result<Self> {
        let file = File::create(path.as_ref())?;
        let mut encoder = Encoder::new(file, width as u16, height as u16, &[])?;
        encoder.set_repeat(Repeat::Infinite)?;
        let font_data = include_bytes!("DejaVuSans.ttf"); // You'll need to provide a font
        let font = FontRef::try_from_slice(font_data).unwrap();
        let scale = PxScale::from(20.0);
        Ok(Self {
            frame: Default::default(),
            width,
            height,
            delay: 20,
            encoder,
            idx: 0,
            font: font.into(),
            font_scale: scale,
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
        self.idx += 1;
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
        let pixels: Vec<u8> = self
            .frame
            .iter()
            .map(|x| (*x - min_px) / (max_px - min_px))
            .map(|t| colormap.eval_continuous(t.into()))
            .map(|c| c.into_array().to_vec())
            .flat_map(|mut c| {
                c.push(255u8);
                c
            })
            .collect();
        let n = self.height;
        let pixels: Vec<_> = (0..n)
            .flat_map(|i| {
                pixels
                    .chunks(n * 4)
                    .skip(i)
                    .step_by(n)
                    .flat_map(|c| c.to_vec())
                    .collect::<Vec<_>>()
            })
            .collect();

        let mut image = RgbaImage::from_vec(self.width as u32, self.height as u32, pixels)
            .expect("failed to create a RGB image");
        draw_text_mut(
            &mut image,
            Rgba([255u8, 255u8, 255u8, 255u8]),
            10,
            10,
            self.font_scale,
            &self.font,
            &format!("#{}", self.idx),
        );
        // draw_guide_lines(&mut image, self.width as u32, self.height as u32);

        let mut frame = GifFrame::from_rgba_speed(
            self.width as u16,
            self.height as u16,
            &mut image.into_raw(),
            10,
        );
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
    T: Send + Sync + PartialOrd + Div<Output = T> + Debug + Copy + Sub<Output = T>,
    f64: From<T>,
    U: UniqueIdentifier<DataType = Vec<T>>,
{
    fn read(&mut self, data: interface::Data<U>) {
        self.frame = data.into_arc();
    }
}
