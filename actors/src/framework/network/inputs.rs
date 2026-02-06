use interface::{Data, TryRead, UniqueIdentifier, Write};

use super::OutputRx;

/// Assign inputs to actors
/* pub trait IntoInputs<'a, T, U, CO, const NO: usize, const NI: usize>
where
    T: 'static + Send + Sync,
    U: 'static + Send + Sync + UniqueIdentifier<DataType = T>,
    CO: 'static + Update + Send + Sync + Write<U>,
{
    /// Creates a new input for 'actor' from the last 'Receiver'
    /*     #[must_use = r#"append ".ok()" to squash the "must use" warning"#]
    fn legacy_into_input<CI, const N: usize>(self, actor: &mut Actor<CI, NO, N>) -> Self
    where
        CI: 'static + Update + Send + io::TryRead<U>,
        Self: Sized; */
    /// Returns an error if there are any unassigned receivers
    ///
    /// Otherwise return the actor with the new output
    fn ok(self) -> crate::Result<&'a mut Actor<CO, NI, NO>>
    where
        Self: Sized;
} */

pub trait AddActorInput<U, C, const NI: usize, const NO: usize>
where
    C: TryRead<U>,
    U: 'static + UniqueIdentifier,
{
    /// Adds a new input to an actor
    fn add_input(&mut self, rx: flume::Receiver<Data<U>>, hash: u64);
}

/// Create new actors inputs
pub trait TryIntoInputs<U, CO, const NO: usize, const N: usize>
where
    U: 'static + UniqueIdentifier,
    CO: 'static + Write<U>,
{
    /// Try to create a new input for 'actor' from the last 'Receiver'
    fn into_input<CI>(self, actor: &mut impl AddActorInput<U, CI, NO, N>) -> Self
    where
        CI: 'static + TryRead<U>,
        Self: Sized;
}

impl<U, CO, const NO: usize, const NI: usize, const N: usize> TryIntoInputs<U, CO, NO, N>
    for std::result::Result<(), OutputRx<U, CO, NI, NO>>
where
    U: 'static + UniqueIdentifier,
    CO: 'static + Write<U>,
{
    // fn into_input<CI, const N: usize>(mut self, actor: &mut Actor<CI, NO, N>) -> Self
    fn into_input<CI>(mut self, actor: &mut impl AddActorInput<U, CI, NO, N>) -> Self
    where
        CI: 'static + TryRead<U>,
    {
        let Err(OutputRx {
            hash, ref mut rxs, ..
        }) = self
        else {
            panic!(r#"Input receivers have been exhausted"#)
        };
        let Some(recv) = rxs.pop() else {
            panic!(r#"Input receivers is empty"#)
        };
        actor.add_input(recv, hash);
        if rxs.is_empty() {
            Ok(())
        } else {
            self
        }
    }
}
// Unique hash for a pair of input/output
/* fn hashio<CO, const NO: usize, const NI: usize>(output_actor: &mut Actor<CO, NI, NO>) -> u64
where
    CO: Update + Send + Sync,
{
    let mut hasher = DefaultHasher::new();
    output_actor.who().hash(&mut hasher);
    let output = output_actor
        .outputs
        .as_mut()
        .and_then(|o| o.last_mut())
        .unwrap();
    output
        .who()
        .split("::")
        .last()
        .unwrap()
        .to_owned()
        .hash(&mut hasher);
    let hash = hasher.finish();
    output.set_hash(hash);
    hash
}
 */
/* impl<'a, T, U, CO, const NO: usize, const NI: usize> IntoInputs<'a, T, U, CO, NO, NI>
    for (&'a mut Actor<CO, NI, NO>, Vec<flume::Receiver<io::Data<U>>>)
where
    T: 'static + Send + Sync,
    U: 'static + Send + Sync + UniqueIdentifier<DataType = T>,
    CO: 'static + Update + Send + io::Write<U>,
{
    fn legacy_into_input<CI, const N: usize>(mut self, actor: &mut Actor<CI, NO, N>) -> Self
    where
        CI: 'static + Update + Send + io::TryRead<U>,
    {
        if let Some(recv) = self.1.pop() {
            actor.add_input(recv, hashio(self.0))
        }
        self
    }
    fn ok(self) -> Result<&'a mut Actor<CO, NI, NO>> {
        if self.1.is_empty() {
            Ok(self.0)
        } else {
            Err(ActorError::OrphanOutput(
                self.0.outputs.as_ref().unwrap().last().unwrap().who(),
                self.0.who(),
            ))
        }
    }
} */
