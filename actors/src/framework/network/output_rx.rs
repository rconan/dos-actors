use std::{
    error::Error,
    fmt::{Debug, Display},
    sync::Arc,
};

use interface::{Data, UniqueIdentifier, Write};

type Rx<U> = flume::Receiver<Data<U>>;

/// Output signature
///
/// [OutputRx] contains the data of an actor output
/// that is necessary to create the associated input for
/// the receiving actor
pub struct OutputRx<U, C, const NI: usize, const NO: usize>
where
    U: UniqueIdentifier,
    C: Write<U>,
{
    pub actor: String,
    pub output: String,
    pub hash: u64,
    pub rxs: Vec<Rx<U>>,
    pub client: Arc<tokio::sync::Mutex<C>>,
}

impl<U, CO, const NO: usize, const NI: usize> std::error::Error for OutputRx<U, CO, NI, NO>
where
    U: 'static + UniqueIdentifier,
    CO: Write<U>,
{
}
/// Type-erased version of [OutputRx]
///
/// [ActorOutputsError ] is used to propagate [OuputRx] error.
#[derive(Debug)]
pub struct ActorOutputsError {
    pub(crate) actor: String,
    pub(crate) output: String,
}
impl Error for ActorOutputsError {}
impl Display for ActorOutputsError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        writeln!(
            f,
            r#"TryIntoInputs for output "{}" of actor "{}", check output multiplex #"#,
            self.output, self.actor
        )
    }
}
impl<U, CO, const NO: usize, const NI: usize> From<OutputRx<U, CO, NI, NO>> for ActorOutputsError
where
    U: 'static + UniqueIdentifier,
    CO: Write<U>,
{
    fn from(value: OutputRx<U, CO, NI, NO>) -> Self {
        ActorOutputsError {
            actor: value.actor,
            output: value.output,
        }
    }
}
impl<U, CO, const NO: usize, const NI: usize> Display for OutputRx<U, CO, NI, NO>
where
    U: 'static + UniqueIdentifier,
    CO: Write<U>,
{
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let OutputRx { actor, output, .. } = self;
        writeln!(
            f,
            r#"TryIntoInputs for output "{}" of actor "{}", check output multiplex #"#,
            output, actor
        )
    }
}
impl<U, CO, const NO: usize, const NI: usize> Debug for OutputRx<U, CO, NI, NO>
where
    U: 'static + UniqueIdentifier,
    CO: Write<U>,
{
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        <Self as Display>::fmt(&self, f)
    }
}
