/*!
# Network framework

The network module defines the interface to link actors to each other.

A single ouput with default parameters is added to an actor and connected to the input of another actor with:
```ignore
actor.add_output().build::<U>().into_input(&mut other)?;
```
and the correspong trail of traits methods is
(solid lines point to the type of the value that a method returns
    and the dashed line points to the type of the input argument ):
![Output to input](https://raw.githubusercontent.com/rconan/dos-actors/nested-models/actors/src/framework/network/out2in.dot.png)

The diagram of the trait bounds on generic parameters is given below:
![Trait bounds](https://raw.githubusercontent.com/rconan/dos-actors/nested-models/actors/src/framework/network/bounds.dot.png)

*/

/// Interface for actors inputs
mod inputs;
pub use inputs::{AddActorInput, TryIntoInputs};

/// Interface for actors outputs
mod outputs;
pub use outputs::{ActorOutput, ActorOutputBuilder, AddActorOutput, AddOuput};

/// Definition of the payload between outputs and inputs
mod output_rx;
pub use output_rx::{ActorOutputsError, OutputRx};

/// Interface for actors log outputs
mod logs;
pub use logs::{IntoLogs, IntoLogsN};
