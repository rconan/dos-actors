use std::env;

use super::{ClientError, Scope};
use eframe::egui;
use egui_plot::{Plot, PlotUi};
use interface::UniqueIdentifier;

const PLOT_SIZE: (f32, f32) = (600f32, 500f32);
const MAX_WINDOW_SIZE: (f32, f32) = (1200f32, 1000f32);

#[derive(Debug, thiserror::Error)]
pub enum GridScopeError {
    #[error("failed to create the scope within the grid")]
    Pin(#[from] ClientError),
}
pub type Result<T> = std::result::Result<T, GridScopeError>;

struct NodeScope {
    indices: (usize, usize),
    scope: Scope,
}

/// Display [Scope]s in a grid like pattern
pub struct GridScope {
    size: (usize, usize),
    scopes: Vec<NodeScope>,
    plot_size: (f32, f32),
    server_ip: String,
    client_address: String,
}
impl GridScope {
    /// Creates a new grid layout for [Scope]s
    ///
    /// `size` sets the number of rows and columns
    pub fn new(size: (usize, usize)) -> Self {
        let (rows, cols) = size;
        let width = MAX_WINDOW_SIZE.0.min(PLOT_SIZE.0 * cols as f32) / cols as f32;
        let height = MAX_WINDOW_SIZE.1.min(PLOT_SIZE.1 * rows as f32) / rows as f32;
        Self {
            size,
            scopes: vec![],
            plot_size: (width, height),
            server_ip: env::var("SCOPE_SERVER_IP").unwrap_or(crate::SERVER_IP.into()),
            client_address: crate::CLIENT_ADDRESS.into(),
        }
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
    fn window_size(&self) -> (f32, f32) {
        let (rows, cols) = self.size;
        let (width, height) = self.plot_size;
        (width * cols as f32, height * rows as f32)
    }
    /// Sets a [Scope] at position `(row,column)` in the grid layout
    pub fn pin<U>(mut self, indices: (usize, usize)) -> Result<Self>
    where
        U: UniqueIdentifier + 'static,
    {
        let (rows, cols) = self.size;
        let (row, col) = indices;
        assert!(
            row < rows,
            "The row index in the scopes grid must be less than {}",
            rows
        );
        assert!(
            col < cols,
            "The columm index in the scopes grid must be less than {}",
            cols
        );
        if let Some(node) = self.scopes.iter_mut().find(|node| node.indices == indices) {
            node.scope.as_mut_signal::<U>()?;
        } else {
            self.scopes.push(NodeScope {
                indices,
                scope: Scope::new()
                    .server_ip(&self.server_ip)
                    .client_address(&self.client_address)
                    .signal::<U>()?,
            });
        }

        /*         self.scopes.push(NodeScope {
            indices,
            scope: Scope::new()
                .server_ip(&self.server_ip)
                .client_address(&self.client_address)
                .signal::<U>()?,
        }); */
        Ok(self)
    }
    /// Display the scope
    pub fn show(mut self) {
        for node in self.scopes.iter_mut() {
            let monitor = node.scope.monitor.take().unwrap();
            tokio::spawn(async move {
                match monitor.join().await {
                    Ok(_) => println!("*** data streaming complete ***"),
                    Err(e) => println!("!!! data streaming error with {:?} !!!", e),
                }
            });
        }
        let native_options = eframe::NativeOptions {
            viewport: egui::ViewportBuilder::default()
                .with_inner_size(egui::Vec2::from(self.window_size())),
            ..Default::default()
        };
        let _ = eframe::run_native(
            "GMT DOS Actors Scope",
            native_options,
            Box::new(|cc| {
                for node in self.scopes.iter_mut() {
                    let scope = &mut node.scope;
                    scope.run(cc.egui_ctx.clone());
                }
                Ok(Box::new(self))
            }),
        );
    }
}

impl eframe::App for GridScope {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        egui::CentralPanel::default().show(ctx, |ui| {
            let (rows, cols) = self.size;
            let style = ui.style_mut();
            style.spacing.item_spacing = egui::vec2(0.0, 0.0);
            for row in 0..rows {
                ui.horizontal(|ui| {
                    for col in 0..cols {
                        self.scopes
                            .iter_mut()
                            .find(|node| node.indices == (row, col))
                            .map(|node| {
                                let plot = Plot::new("Scope")
                                    .legend(Default::default())
                                    .width(self.plot_size.0)
                                    .height(self.plot_size.1)
                                    .set_margin_fraction(egui::Vec2::from((0.05, 0.05)));
                                plot.show(ui, |plot_ui: &mut PlotUi| {
                                    for signal in &mut node.scope.signals {
                                        signal.plot_ui(plot_ui, node.scope.n_sample)
                                    }
                                });
                            });
                    }
                });
            }
        });
    }
}
