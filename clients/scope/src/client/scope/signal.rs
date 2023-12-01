use std::{
    ops::Deref,
    sync::{Arc, RwLock},
};

use eframe::{
    egui::{
        self,
        plot::{BoxElem, BoxPlot, BoxSpread, Line, Plot, PlotImage, PlotPoint, PlotUi, Text},
        RichText,
    },
    emath::Align2,
};
use interface::{Data, UniqueIdentifier};
// use tracing::warn;

mod data;
use data::SignalData;

use self::data::Quantiles;

type D<U> = Data<crate::payload::ScopeData<U>>;
pub(super) struct Signal<U>
where
    U: UniqueIdentifier,
{
    rx: Option<flume::Receiver<D<U>>>,
    data: Arc<RwLock<Option<SignalData>>>,
}
impl<U> Signal<U>
where
    U: UniqueIdentifier,
{
    pub fn new(rx: Option<flume::Receiver<D<U>>>) -> Self {
        Self {
            rx,
            data: Arc::new(RwLock::new(None)),
        }
    }
}

pub(super) trait SignalProcessing {
    fn run(&mut self, ctx: egui::Context);
    fn plot_ui(&self, ui: &mut PlotUi);
    fn plot_stats_ui(&self, ctx: &egui::Context);
    // fn minmax(&self) -> Option<(f64, f64)>;
}

impl<U> SignalProcessing for Signal<U>
where
    U: UniqueIdentifier + 'static,
{
    fn run(&mut self, mut ctx: egui::Context) {
        let rx = self.rx.take().unwrap();
        let data = self.data.clone();
        tokio::spawn(async move {
            while let Some(wrap) = rx.recv().ok() {
                let payload = wrap.deref();
                data.write()
                    .unwrap()
                    .get_or_insert(SignalData::from(payload))
                    .add_payload(&mut ctx, payload);
                ctx.request_repaint();
            }
            // warn!("{name}: stream ended");
            drop(rx);
        });
    }
    fn plot_ui(&self, ui: &mut PlotUi) {
        if let Some(data) = self.data.read().unwrap().as_ref() {
            match data {
                SignalData::Signal { tag, points, .. } => {
                    let line = Line::new(points.clone()).name(tag);
                    ui.line(line);
                }
                SignalData::Image {
                    size,
                    time,
                    texture,
                    ..
                } => {
                    texture.as_ref().map(|texture| {
                        let image = PlotImage::new(
                            texture,
                            PlotPoint::new(0., 0.),
                            (2f32 * size[0] as f32 / size[1] as f32, 2f32),
                        );
                        ui.image(image);
                        ui.text(
                            Text::new(
                                PlotPoint::new(-1., 1.),
                                RichText::new(format!("{time:.3}s")).size(14f32).strong(),
                            )
                            .anchor(Align2::LEFT_TOP),
                        );
                    });
                }
                SignalData::Signals(signals) => {
                    signals.iter().enumerate().for_each(|(i, signal)| {
                        if let SignalData::Signal { tag, points, .. } = signal {
                            let line = Line::new(points.clone()).name(format!("{tag} #{i}"));
                            ui.line(line);
                        }
                    })
                }
            }
        }
    }

    fn plot_stats_ui(&self, ctx: &egui::Context) {
        if let Some(data) = self.data.read().unwrap().as_ref() {
            match data {
                SignalData::Signal { .. } => {
                    unimplemented!();
                }
                SignalData::Image { quantiles, .. } => {
                    if let &Some(Quantiles {
                        minimum,
                        lower_whisker,
                        quartile1,
                        median,
                        quartile3,
                        upper_whisker,
                        maximum,
                    }) = quantiles
                    {
                        egui::TopBottomPanel::top("top")
                            .min_height(100.)
                            .show(ctx, |ui| {
                                Plot::new("Box Plot")
                                    .include_x(minimum)
                                    .include_x(maximum)
                                    .include_y(75.)
                                    .show(ui, |plot_ui: &mut PlotUi| {
                                        plot_ui.box_plot(BoxPlot::new(vec![BoxElem::new(
                                            0.,
                                            BoxSpread::new(
                                                lower_whisker,
                                                quartile1,
                                                median,
                                                quartile3,
                                                upper_whisker,
                                            ),
                                        )
                                        .box_width(40.)
                                        .whisker_width(50.)
                                        .horizontal()]));
                                    });
                            });
                    }
                }
                SignalData::Signals(_) => todo!(),
            }
        }
    }
    /*     fn minmax(&self) -> Option<(f64, f64)> {
        if let Some(data) = self.data.read().unwrap().as_ref() {
            match data {
                SignalData::Signal { .. } => None,
                SignalData::Image { minmax, .. } => minmax.clone(),
            }
        } else {
            None
        }
    } */
}
