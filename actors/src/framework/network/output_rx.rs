use std::{
    error::Error,
    fmt::{Debug, Display},
    sync::Arc,
};

use interface::{Data, TryWrite, UniqueIdentifier};

type Rx<U> = flume::Receiver<Data<U>>;

/// Output signature
///
/// [OutputRx] contains the data of an actor output
/// that is necessary to create the associated input for
/// the receiving actor
#[non_exhaustive]
pub enum OutputRx<U, C, const NI: usize, const NO: usize>
where
    U: UniqueIdentifier,
    C: TryWrite<U>,
{
    Output {
        actor: String,
        output: String,
        hash: u64,
        rxs: Vec<Rx<U>>,
        client: Arc<tokio::sync::Mutex<C>>,
    },
    ExhaustedRxs,
    EmptyRxs,
}
impl<U, CO, const NO: usize, const NI: usize> std::error::Error for OutputRx<U, CO, NI, NO>
where
    U: 'static + UniqueIdentifier,
    CO: TryWrite<U>,
{
}
/// Type-erased version of [OutputRx]
///
/// [ActorOutputsError ] is used to propagate [OuputRx] error.
#[derive(Debug, Default)]
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
    CO: TryWrite<U>,
{
    fn from(value: OutputRx<U, CO, NI, NO>) -> Self {
        if let OutputRx::Output { actor, output, .. } = value {
            ActorOutputsError { actor, output }
        } else {
            Default::default()
        }
    }
}
impl<U, CO, const NO: usize, const NI: usize> Display for OutputRx<U, CO, NI, NO>
where
    U: 'static + UniqueIdentifier,
    CO: TryWrite<U>,
{
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            OutputRx::Output { actor, output, .. } => writeln!(
                f,
                r#"TryIntoInputs for output "{}" of actor "{}", check output multiplex #"#,
                output, actor
            ),
            Self::ExhaustedRxs => writeln!(f, r#"Input receivers have been exhausted"#),
            Self::EmptyRxs => writeln!(f, r#"Input receivers is empty"#),
        }
    }
}
impl<U, CO, const NO: usize, const NI: usize> Debug for OutputRx<U, CO, NI, NO>
where
    U: 'static + UniqueIdentifier,
    CO: TryWrite<U>,
{
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        <Self as Display>::fmt(self, f)
    }
}
