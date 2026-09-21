# GMT Actors Model

A GMT integrated model is a collection of actors, each actor executing a specific task or set of tasks and exchanging data at predefined sampling rates.

The GMT Actors Model is distributed among 3 crates: `gmt_dos-actors`, `gmt_dos-clients` and `gmt_dos_actors-clients_interface`.

`gmt_dos-actors` implements the actor model including the methods to send and receive data to and from actors and the higher level abstraction of a model.

The interface between a client and the inputs and outputs of an actor is defined in the `gmt_dos-actors-clients_interface` crate.

The crate `gmt_dos-clients` provides a set of predefined clients.

To use `gmt_dos-actors`, add it to your list of dependencies with 
```
cargo add gmt_dos-actors
```
and import the contents of the prelude module:
```rust,,no_run,noplayground
use gmt_dos_actors::prelude::*;
```

To use some of the clients in `gmt_dos-clients`, add the crate to your list of dependencies with 
```
cargo add gmt_dos-clients
```
If you are only looking for the `gmt_dos-actors` interface, you can instead do
```
cargo add gmt_dos-actors-clients_interface --rename interface
```

|||||
|-|-|-|-|
|`gmt_dos-actors`| [crates.io](https://crates.io/crates/gmt_dos-actors) | [docs.rs](https://docs.rs/gmt_dos-actors/) | [github](https://github.com/rconan/dos-actors) |
|`gmt_dos-actors-clients_interface`| [crates.io](https://crates.io/crates/gmt_dos-actors-clients_interface) | [docs.rs](https://docs.rs/gmt_dos-actors-clients_interface/) | [github](https://github.com/rconan/dos-actors/tree/main/interface) |
|`gmt_dos-clients`| [crates.io](https://crates.io/crates/gmt_dos-clients) | [docs.rs](https://docs.rs/gmt_dos-clients/) | [github](https://github.com/rconan/dos-actors/tree/main/clients/core) |
