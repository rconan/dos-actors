use std::{
    env, fmt,
    ops::{Div, Sub},
    path::{Path, PathBuf},
    sync::Arc,
};

use colorous::CIVIDIS;
use image::{Rgba, RgbaImage};
use imageproc::drawing::draw_cross_mut;
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
    filter: Option<F>,
    crosses: Option<Vec<(i32, i32)>>,
}
impl<T, F: Fn(&T) -> T> Frame<T, F> {
    /// Creates a new GIF encoder
    ///
    /// The `width` and `height` of the image must match the frame size
    pub fn new<P: AsRef<Path>>(path: P, size: usize) -> Self {
        let data_path = env::var("DATA_REPO").unwrap_or(".".into());
        let path = Path::new(&data_path).join(path);
        Self {
            path,
            frame: Default::default(),
            size,
            image: Default::default(),
            filter: None,
            crosses: None,
        }
    }
    pub fn save(&self) -> Result<()> {
        Ok(self.image.save(self.path.as_path())?)
    }
    pub fn filter(mut self, filter: F) -> Self {
        self.filter = Some(filter);
        self
    }
    pub fn cross(mut self, cross: (i32, i32)) -> Self {
        self.crosses.get_or_insert(vec![]).push(cross);
        self
    }
}
impl<T, F> Update for Frame<T, F>
where
    T: Send + Sync + PartialOrd + Div<Output = T> + fmt::Debug + Copy + Sub<Output = T>,
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
        self.save()
            .expect(&format!("failed to write frame to {:?}", self.path));
    }
}
impl<T, F, U> Read<U> for Frame<T, F>
where
    T: Send + Sync + PartialOrd + Div<Output = T> + fmt::Debug + Copy + Sub<Output = T>,
    f64: From<T>,
    U: UniqueIdentifier<DataType = Vec<T>>,
    F: Send + Sync + Fn(&T) -> T,
{
    fn read(&mut self, data: interface::Data<U>) {
        self.frame = data.into_arc();
    }
}
