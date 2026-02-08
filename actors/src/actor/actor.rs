use super::io::{Input, InputObject, OutputObject};
use crate::{
    framework::network::{ActorOutput, ActorOutputBuilder, AddActorInput, AddActorOutput}, Result,
};
use futures::{future::join_all, stream::FuturesUnordered};
use interface::{Data, TryRead, TryUpdate, UniqueIdentifier, Who};
use std::{
    fmt::{self, Debug},
    sync::Arc,
};
use tokio::sync::Mutex;

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
/// Actor model implementation
pub struct Actor<C, const NI: usize = 1, const NO: usize = 1>
where
    C: TryUpdate,
{
    #[cfg_attr(feature = "serde", serde(skip))]
    pub(crate) inputs: Option<Vec<Box<dyn InputObject>>>,
    #[cfg_attr(feature = "serde", serde(skip))]
    pub(crate) outputs: Option<Vec<Box<dyn OutputObject>>>,
    #[cfg_attr(
        feature = "serde",
        serde(
            with = "super::serde_with",
            bound(
                serialize = "C: serde::Serialize",
                deserialize = "C: serde::Deserialize<'de>"
            )
        )
    )]
    pub(crate) client: Arc<Mutex<C>>,
    pub(crate) name: Option<String>,
    pub(crate) image: Option<String>,
}

/// Clone trait implementation
///
/// Cloning an actor preserves the state of the client
/// but inputs and outputs are deleted and need to be reset
impl<C: TryUpdate, const NI: usize, const NO: usize> Clone for Actor<C, NI, NO> {
    fn clone(&self) -> Self {
        Self {
            inputs: None,
            outputs: None,
            client: self.client.clone(),
            name: self.name.clone(),
            image: self.image.clone(),
        }
    }
}

impl<C, const NI: usize, const NO: usize> fmt::Display for Actor<C, NI, NO>
where
    C: TryUpdate,
{
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        writeln!(f, "{}:", self.who().to_uppercase())?;
        if let Some(inputs) = self.inputs.as_ref() {
            writeln!(f, " - inputs  #{:>1}:", inputs.len())?;
            for (k, input) in self.inputs.as_ref().unwrap().iter().enumerate() {
                writeln!(f, "   {}. {}", 1 + k, input)?;
            }
        }

        if let Some(outputs) = self.outputs.as_ref() {
            writeln!(f, " - outputs #{:>1}:", outputs.len())?;
            for (k, output) in self.outputs.as_ref().unwrap().iter().enumerate() {
                /*                     writeln!(
                    f,
                    "   {}. {} (#{}, bootstrap)",
                    1 + k,
                    (*output).who(),
                    output.len()
                )?; */
                writeln!(f, "   {}. {}", 1 + k, output)?;
            }
        }

        Ok(())
    }
}
impl<C, const NI: usize, const NO: usize> fmt::Debug for Actor<C, NI, NO>
where
    C: TryUpdate + Debug,
{
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Actor")
            .field("inputs", &self.inputs)
            .field("outputs", &self.outputs)
            .field("client", &self.client)
            .field("name", &self.name)
            .field("image", &self.image)
            .finish()
    }
}

impl<C, const NI: usize, const NO: usize> From<C> for Actor<C, NI, NO>
where
    C: TryUpdate,
{
    /// Creates a new actor for the client
    fn from(client: C) -> Self {
        Actor::new(Arc::new(Mutex::new(client)))
    }
}
impl<C, S, const NI: usize, const NO: usize> From<(C, S)> for Actor<C, NI, NO>
where
    C: TryUpdate,
    S: Into<String>,
{
    /// Creates a new named actor for the client
    fn from((client, name): (C, S)) -> Self {
        let mut actor = Actor::new(Arc::new(Mutex::new(client)));
        actor.name = Some(name.into());
        actor
    }
}
impl<C, const NI: usize, const NO: usize> Who<C> for Actor<C, NI, NO>
where
    C: TryUpdate,
{
    fn who(&self) -> String {
        self.name
            .as_ref()
            .cloned()
            .unwrap_or_else(|| std::any::type_name::<C>().into())
    }
}

impl<C, const NI: usize, const NO: usize> Actor<C, NI, NO>
where
    C: TryUpdate,
{
    /// Creates a new [Actor] for the given client
    pub fn new(client: Arc<Mutex<C>>) -> Self {
        Self {
            inputs: None,
            outputs: None,
            client,
            name: None,
            image: None,
        }
    }
    pub fn name<S: Into<String>>(self, name: S) -> Self {
        Self {
            name: Some(name.into()),
            ..self
        }
    }
    pub fn image<S: Into<String>>(self, image: S) -> Self {
        Self {
            image: Some(image.into()),
            ..self
        }
    }
    /// Returns a pointer to the actor's client
    pub fn client(&self) -> Arc<Mutex<C>> {
        Arc::clone(&self.client)
    }
    /// Gathers all the inputs from other [Actor] outputs
    pub(super) async fn collect(&mut self) -> Result<&mut Self> {
        if let Some(inputs) = &mut self.inputs {
            let futures: FuturesUnordered<_> =
                inputs.iter_mut().map(|input| input.recv()).collect();
            join_all(futures)
                .await
                .into_iter()
                .collect::<Result<Vec<_>>>()?;
        }
        Ok(self)
    }
    /// Sends the outputs to other [Actor] inputs
    pub(super) async fn distribute(&mut self) -> Result<&mut Self> {
        if let Some(outputs) = &mut self.outputs {
            let futures: FuturesUnordered<_> =
                outputs.iter_mut().map(|output| output.send()).collect();
            join_all(futures)
                .await
                .into_iter()
                .collect::<Result<Vec<_>>>()?;
        }
        Ok(self)
    }
    /// Invokes outputs senders
    pub(super) async fn bootstrap(&mut self) -> Result<bool> {
        if let Some(outputs) = &mut self.outputs {
            async fn inner(outputs: &mut Vec<Box<dyn OutputObject>>) -> Result<Vec<()>> {
                let futures: Vec<_> = outputs
                    .iter_mut()
                    .filter(|output| output.bootstrap())
                    .inspect(|output| {
                        interface::print_info(
                            format!("{} bootstrapped", output.highlight()),
                            None::<&dyn std::error::Error>,
                        )
                    })
                    .map(|output| output.send())
                    .collect();
                join_all(futures)
                    .await
                    .into_iter()
                    .collect::<Result<Vec<_>>>()
            }
            if NO >= NI {
                inner(outputs).await.map(|result| !result.is_empty())
            } else {
                let mut a = true;
                for _ in 0..NI / NO {
                    a = a && inner(outputs).await.map(|result| !result.is_empty())?;
                }
                Ok(a)
            }
        } else {
            Ok(false)
        }
    }
}

/* impl<C, const NI: usize, const NO: usize> Actor<C, NI, NO>
where
    C: 'static + TryUpdate + Send + Sync,
{
    /// Adds a new output
    pub fn add_output(&mut self) -> ActorOutput<'_, Actor<C, NI, NO>> {
        ActorOutput::new(self, ActorOutputBuilder::new(1))
    }
} */
impl<'a, C, const NI: usize, const NO: usize> AddActorOutput<'a, C, NI, NO> for Actor<C, NI, NO>
where
    C: TryUpdate + 'static,
{
    /// Adds a new output
    fn add_output(&'a mut self) -> ActorOutput<'a, Actor<C, NI, NO>> {
        ActorOutput::new(self, ActorOutputBuilder::new(1))
    }
}
impl<U, C, const NI: usize, const NO: usize> AddActorInput<U, C, NI, NO> for Actor<C, NI, NO>
where
    C: TryRead<U> + 'static,
    U: 'static + UniqueIdentifier,
{
    /// Adds an input to an actor
    fn add_input(&mut self, rx: flume::Receiver<Data<U>>, hash: u64) {
        let input: Input<C, U, NI> = Input::new(rx, self.client.clone(), hash);
        if let Some(ref mut inputs) = self.inputs {
            inputs.push(Box::new(input));
        } else {
            self.inputs = Some(vec![Box::new(input)]);
        }
    }
}
/*
impl<C, const NI: usize, const NO: usize> Drop for Actor<C, NI, NO>
where
    C: TryUpdate + Send,
{
    fn drop(&mut self) {
        log::info!("{} dropped!", self.who());
    }
}
*/
