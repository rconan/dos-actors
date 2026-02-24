use std::{
    env, fmt,
    ops::{Div, Sub},
    path::{Path, PathBuf},
    sync::Arc,
};

use ab_glyph::{FontArc, FontRef};
use colorous::CIVIDIS;
use image::{Rgba, RgbaImage};
use imageproc::drawing::{draw_cross_mut, draw_text_mut};
use interface::{Read, UniqueIdentifier, Update};

use super::GifError;

type Result<T> = std::result::Result<T, GifError>;
pub struct Frame<T, F = fn(&T) -> T>
where
    F: Fn(&T) -> T,
{
    path: PathBuf,
    frame: Arc<Vec<T>>,
    size: usize,
    image: RgbaImage,
    idx: usize,
    font: FontArc,
    filter: Option<F>,
    crosses: Option<Vec<(i32, i32)>>,
    autosave: bool,
}
impl<T, F: Fn(&T) -> T> Frame<T, F> {
    /// Creates a new image encoder
    ///
    /// The image is saved at the given location and is of the given size
    pub fn new<P: AsRef<Path>>(path: P, size: usize) -> Self {
        let data_path = env::var("DATA_REPO").unwrap_or(".".into());
        let path = Path::new(&data_path).join(path);
        let font_data = include_bytes!("../DejaVuSans.ttf"); // You'll need to provide a font
        let font = FontRef::try_from_slice(font_data).unwrap();
        Self {
            path,
            frame: Default::default(),
            size,
            image: Default::default(),
            idx: 0,
            font: font.into(),
            filter: None,
            crosses: None,
            autosave: true,
        }
    }
    /// Enables/disables auto-save property (enabled per default)
    pub fn autosave(mut self, flag: bool) -> Self {
        self.autosave = flag;
        self
    }
    /// Saves the the image
    pub fn save(&self) -> Result<()> {
        Ok(self.image.save(self.path.as_path())?)
    }
    /// Sets the image filter
    pub fn filter(mut self, filter: F) -> Self {
        self.filter = Some(filter);
        self
    }
    /// Add a marker to the image at the (x,y) pixel
    pub fn cross(mut self, cross: (i32, i32)) -> Self {
        self.crosses.get_or_insert(vec![]).push(cross);
        self
    }
    /// Returns the image
    pub fn image(&self) -> &RgbaImage {
        &self.image
    }
    /// Returns the path to the image
    pub fn path(&self) -> &Path {
        self.path.as_path()
    }
}
impl<T, F> Update for Frame<T, F>
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
    F: Send + Sync + Fn(&T) -> T,
{
    fn update(&mut self) {
        if let Some(filter) = self.filter.as_ref() {
            self.frame = Arc::new(self.frame.iter().map(|x| (filter)(x)).collect());
        }
        let max_px = *self
            .frame
            .iter()
            .max_by(|&a, &b| a.partial_cmp(b).unwrap())
            .unwrap();
        // dbg!(max_px);
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

        let n = self.size;
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
        let w = self.frame.len() / n;

        self.image =
            RgbaImage::from_vec(w as u32, n as u32, pixels).expect("failed to create a RGBA image");
        draw_text_mut(
            &mut self.image,
            Rgba([255u8, 255u8, 255u8, 255u8]),
            10,
            10,
            16.,
            &self.font,
            &format!(
                "{} #{}",
                self.path
                    .with_extension("")
                    .file_name()
                    .unwrap()
                    .to_str()
                    .unwrap(),
                self.idx
            ),
        );
        draw_text_mut(
            &mut self.image,
            Rgba([255u8, 255u8, 255u8, 255u8]),
            10,
            28,
            12.0,
            &self.font,
            &format!("[{:.3e},{:.3e}]", min_px, max_px),
        );
        draw_cross_mut(
            &mut self.image,
            Rgba([255u8, 0, 0, 255u8]),
            w as i32 / 2,
            n as i32 / 2,
        );
        if let Some(crosses) = self.crosses.as_ref() {
            for cross in crosses {
                let (y, x) = cross;
                draw_cross_mut(
                    &mut self.image,
                    Rgba([0u8, 0u8, 0u8, 255u8]),
                    w as i32 / 2 + x,
                    n as i32 / 2 + y,
                );
            }
        } // draw_guide_lines(&mut self.image, self.width as u32, self.height as u32);
        if self.autosave {
            self.save()
                .expect(&format!("failed to write frame to {:?}", self.path));
        }
        self.idx += 1;
    }
}
impl<T, F, U> Read<U> for Frame<T, F>
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
    U: UniqueIdentifier,
    F: Send + Sync + Fn(&T) -> T,
    Vec<T>: From<interface::Data<U>>,
{
    fn read(&mut self, data: interface::Data<U>) {
        self.frame = Vec::<T>::from(data).into();
    }
}
