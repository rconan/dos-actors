use gmt_dos_clients_transceiver::{Monitor, On, Transceiver, Transmitter, TransmitterBuilder};
use interface::{Data, Read, UniqueIdentifier};

use crate::{
    GmtScope, ImageScope, ImageScopeKind,
    payload::{Payload, ScopeData},
};

use super::XScope;

/// Server for image display scope
pub type Shot<FU> = XScope<FU, ImageScope>;
/// Server for GMT scope
pub type GmtShot<FU> = XScope<FU, GmtScope>;

impl<'a, FU, K> super::Builder<'a, FU, K>
where
    FU: UniqueIdentifier + 'static,
    K: ImageScopeKind,
{
    /// Build the [Shot]
    pub fn build(self) -> Result<XScope<FU, K>, super::ServerError> {
        Ok(XScope {
            tx: if self.frame_by_frame {
                TransmitterBuilder::new(self.address).capacity(0).build()?
            } else {
                Transceiver::transmitter(self.address)?
            }
            .run(self.monitor.unwrap()),
            size: self.size.unwrap(),
            minmax: self.minmax,
            scale: self.scale,
            tau: self.tau.unwrap_or(1f64),
            idx: Some(0),
            kind: std::marker::PhantomData,
        })
    }
    /// Sends images one by one
    ///
    /// The default behavior is to send a buffer of images
    /// whenever possible
    pub fn frame_by_frame(mut self) -> Self {
        self.frame_by_frame = true;
        self
    }
    /// Sets the minimum and maximum values of the image colormap
    pub fn minmax(mut self, minmax: (f64, f64)) -> Self {
        self.minmax = Some(minmax);
        self
    }
}

/* /// [Shot](crate::Shot) server
///
/// Wraps a signal into the scope payload before sending it to a [XScope](crate::XScope)
#[derive(Debug)]
pub struct Shot<FU>
where
    FU: UniqueIdentifier,
{
    tx: Transceiver<ScopeData<FU>, Transmitter, On>,
    size: [usize; 2],
    minmax: Option<(f64, f64)>,
    scale: Option<f64>,
}
 */
impl<FU, K> XScope<FU, K>
where
    FU: UniqueIdentifier + 'static,
    <FU as UniqueIdentifier>::DataType: Send + Sync + serde::Serialize,
    K: ImageScopeKind,
{
    /// Creates a [Builder](super::Builder)
    pub fn builder(monitor: &mut Monitor, size: [usize; 2]) -> super::Builder<FU, K> {
        super::Builder {
            monitor: Some(monitor),
            size: Some(size),
            ..Default::default()
        }
    }
}

impl<T, FU> Read<FU> for Shot<FU>
where
    FU: UniqueIdentifier<DataType = Vec<T>>,
    T: Copy,
    f64: From<T>,
{
    fn read(&mut self, data: Data<FU>) {
        let payload = Payload::image(data, self.tau, self.size, self.minmax, self.scale)
            .expect("failed to create payload from data");
        <Transceiver<ScopeData<FU>, Transmitter, On> as Read<ScopeData<FU>>>::read(
            &mut self.tx,
            Data::new(payload),
        );
    }
}

impl<T, FU> Read<FU> for GmtShot<FU>
where
    FU: UniqueIdentifier<DataType = (Vec<T>, Vec<bool>)>,
    T: Copy,
    f64: From<T>,
{
    fn read(&mut self, data: Data<FU>) {
        let payload = Payload::gmt(data, self.tau, self.size, self.minmax, self.scale)
            .expect("failed to create payload from data");
        <Transceiver<ScopeData<FU>, Transmitter, On> as Read<ScopeData<FU>>>::read(
            &mut self.tx,
            Data::new(payload),
        );
    }
}
