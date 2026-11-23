use crate::SignalToFilter;
use interface::{Data, Read, Update, Write, UID};
use rand_distr::{Distribution, Normal};

pub struct Filter {
    data: f64,
    noise: Normal<f64>,
    step: usize,
}
impl Default for Filter {
    fn default() -> Self {
        Self {
            data: 0f64,
            noise: Normal::new(0.3, 0.05).unwrap(),
            step: 0,
        }
    }
}
impl Update for Filter {
    fn update(&mut self) {
        self.data += 0.05
            * (2. * std::f64::consts::PI * self.step as f64 * (1e3f64 * 2e-2).recip()).sin()
            + self.noise.sample(&mut rand::rng());
        self.step += 1;
    }
}
impl Read<SignalToFilter> for Filter {
    fn read(&mut self, data: Data<SignalToFilter>) {
        self.data = *data;
    }
}

#[derive(UID)]
#[uid(data = f64)]
pub enum FilterToSink {}
impl Write<FilterToSink> for Filter {
    fn write(&mut self) -> Option<Data<FilterToSink>> {
        Some(Data::new(self.data))
    }
}

#[derive(UID)]
#[uid(data = f64)]
pub enum FilterToSampler {}
impl Write<FilterToSampler> for Filter {
    fn write(&mut self) -> Option<Data<FilterToSampler>> {
        Some(Data::new(self.data))
    }
}

#[derive(UID)]
#[uid(data = f64)]
pub enum FilterToDifferentiator {}
impl Write<FilterToDifferentiator> for Filter {
    fn write(&mut self) -> Option<Data<FilterToDifferentiator>> {
        Some(Data::new(self.data))
    }
}
