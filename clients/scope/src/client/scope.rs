use std::{env, marker::PhantomData};

use eframe::egui;
use egui_plot::{Corner, Legend, Plot, PlotUi};
use gmt_dos_clients_transceiver::{CompactRecvr, Monitor, Transceiver, TransceiverError};
use interface::UniqueIdentifier;
use tokio::task::JoinError;
use tracing::debug;

mod signal;
use signal::{Signal, SignalProcessing};

use crate::{GmtScope, ImageScope, PlotScope, ScopeKind};

#[derive(Debug, thiserror::Error)]
pub enum ClientError {
    #[error("failed to build transceiver")]
    Transceiver(#[from] TransceiverError),
    #[error("some task didn't terminate successfully")]
    Join(#[from] JoinError),
}
pub type Result<T> = std::result::Result<T, ClientError>;

/// Data scope client
pub struct XScope<K = PlotScope>
where
    K: ScopeKind,
{
    server_ip: String,
    client_address: String,
    pub(super) monitor: Option<Monitor>,
    pub(super) signals: Vec<Box<dyn SignalProcessing>>,
    pub(super) n_sample: Option<usize>,
    min_recvr: Option<CompactRecvr>,
    name: String,
    kind: PhantomData<K>,
}
impl<K: ScopeKind> XScope<K> {
    /// Creates a new scope
    ///
    /// A scope is build from both the server IP and the client internet socket addresses
    pub fn new() -> Self {
        Self {
            monitor: Some(Monitor::new()),
            server_ip: env::var("SCOPE_SERVER_IP").unwrap_or(crate::SERVER_IP.into()),
            client_address: crate::CLIENT_ADDRESS.into(),
            signals: Vec::new(),
            n_sample: None,
            min_recvr: None,
            name: String::from("GMT DOS Actors Scope"),
            kind: PhantomData,
        }
    }
    /// Sets the number of samples to be displayed
    pub fn n_sample(mut self, n_sample: usize) -> Self {
        self.n_sample = Some(n_sample);
        self
    }
    /// Sets the server IP address
    pub fn server_ip<S: Into<String>>(mut self, server_ip: S) -> Self {
        self.server_ip = server_ip.into();
        self
    }
    /// Sets the client internet socket address
    pub fn client_address<S: Into<String>>(mut self, client_address: S) -> Self {
        self.client_address = client_address.into();
        self
    }
    /// Sets the scope name
    pub fn name<S: Into<String>>(mut self, name: S) -> Self {
        self.name = name.into();
        self
    }
    /// Adds a signal to the scope
    pub fn signal<U>(mut self) -> Result<Self>
    where
        U: UniqueIdentifier + 'static,
    {
        let rx = if let Some(min_recvr) = self.min_recvr.as_ref() {
            min_recvr.spawn(&self.server_ip)?
        } else {
            let recvr = Transceiver::<crate::payload::ScopeData<U>>::receiver(
                &self.server_ip,
                &self.client_address,
            )?;
            self.min_recvr = Some(CompactRecvr::from(&recvr));
            recvr
        }
        .run(self.monitor.as_mut().unwrap())
        .take_channel_receiver();
        self.signals.push(Box::new(Signal::new(rx)));
        Ok(self)
    }
    pub fn as_mut_signal<U>(&mut self) -> Result<&mut Self>
    where
        U: UniqueIdentifier + 'static,
    {
        let rx = if let Some(min_recvr) = self.min_recvr.as_ref() {
            min_recvr.spawn(&self.server_ip)?
        } else {
            let recvr = Transceiver::<crate::payload::ScopeData<U>>::receiver(
                &self.server_ip,
                &self.client_address,
            )?;
            self.min_recvr = Some(CompactRecvr::from(&recvr));
            recvr
        }
        .run(self.monitor.as_mut().unwrap())
        .take_channel_receiver();
        self.signals.push(Box::new(Signal::new(rx)));
        Ok(self)
    }
    /// Initiates data acquisition
    pub fn run(&mut self, ctx: egui::Context) {
        debug!("scope run");
        self.signals.iter_mut().for_each(|signal| {
            let _ = signal.run(ctx.clone());
        });
    }
    /// Takes ownership of [Monitor]
    pub fn take_monitor(&mut self) -> Monitor {
        self.monitor.take().unwrap()
    }
}

impl<K> XScope<K>
where
    XScope<K>: eframe::App,
    K: ScopeKind + 'static,
{
    /// Display the scope
    pub fn show(mut self) {
        let monitor = self.monitor.take().unwrap();
        tokio::spawn(async move {
            match monitor.join().await {
                Ok(_) => println!("*** data streaming complete ***"),
                Err(e) => println!("!!! data streaming error with {:?} !!!", e),
            }
        });
        let native_options = eframe::NativeOptions {
            viewport: egui::ViewportBuilder::default()
                .with_inner_size(egui::Vec2::from(<K as ScopeKind>::window_size())),
            ..Default::default()
        };
        let _ = eframe::run_native(
            &self.name.clone(),
            native_options,
            Box::new(|cc| {
                Ok(Box::new({
                    self.run(cc.egui_ctx.clone());
                    self
                }))
            }),
        );
    }
}

/// Signal plotting scope
pub type Scope = XScope<PlotScope>;

impl eframe::App for Scope {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        egui::CentralPanel::default().show(ctx, |ui| {
            let plot = Plot::new("Scope").legend(Legend::default().position(Corner::LeftTop));
            plot.show(ui, |plot_ui: &mut PlotUi| {
                for signal in &mut self.signals {
                    // plot_ui.line(signal.line());
                    signal.plot_ui(plot_ui, self.n_sample)
                }
            });
        });
    }
}

/// Image display scope
pub type Shot = XScope<ImageScope>;

impl eframe::App for Shot {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        egui::CentralPanel::default().show(ctx, |ui| {
            let plot = Plot::new("Scope")
                //.show_axes([false; 2])
                .show_x(false)
                .show_y(false)
                .allow_scroll(false)
                .data_aspect(1f32);
            plot.show(ui, |plot_ui: &mut PlotUi| {
                for signal in &mut self.signals {
                    // plot_ui.line(signal.line());
                    signal.plot_ui(plot_ui, None)
                }
            });
        });
    }
}

/// GMT scope
///
/// Image display scope which data is masked by the GMT exit pupil mask
pub type GmtShot = XScope<GmtScope>;

impl eframe::App for GmtShot {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        for signal in &mut self.signals {
            // plot_ui.line(signal.line());
            signal.plot_stats_ui(ctx)
        }
        egui::CentralPanel::default().show(ctx, |ui| {
            let plot = Plot::new("Scope")
                // .show_axes([false; 2])
                .show_x(false)
                .show_y(false)
                // .allow_drag(false)
                .allow_scroll(false)
                .data_aspect(1f32);
            // .view_aspect(1f32);
            plot.show(ui, |plot_ui: &mut PlotUi| {
                for signal in &mut self.signals {
                    // plot_ui.line(signal.line());
                    signal.plot_ui(plot_ui, None)
                }
            });
        });
    }
}
