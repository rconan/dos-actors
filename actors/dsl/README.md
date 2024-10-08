# `actorscript`

[![Crates.io](https://img.shields.io/crates/v/gmt_dos-actors_dsl.svg)](https://crates.io/crates/gmt_dos-actors_dsl)
[![Documentation](https://docs.rs/gmt_dos-actors_dsl/badge.svg)](https://docs.rs/gmt_dos-actors_dsl/)

A scripting micro-language for [gmt_dos-actors].

The `actorscript` procedural macro is a [Domain Specific Language] to write [gmt_dos-actors] models.

`actorscript` parses **flows**.
A **flow** consists in a sampling rate followed by a **chain**.
A **chain** is a series of pairs of actor's client and an actor output separated by the token `->`.

As an example:
```rust
actorscript! {
    1: a[A2B] -> b
};
```
is a **flow** connecting the output `A2B` of client `a` to an input of client `b` at the nominal sampling rate.
This example will be expanded by the compiler to
```rust
let mut a: Actor<_,1,1> = a.into();
let mut b: Actor<_,1,1> = b.into();
a.add_output().build::<A2B>().into_input(&mut b)?;
let model = model!(a,b).name("model").flowchart().check()?;
```
For the code above to compile successfully, the traits [`Write<A2B>`] and [`Read<A2B>`]
must have been implemented for the clients `a` and `b`, respectively.

The [gmt_dos-actors] model is written in the `completed` state meaning that the model is automatically run to completion

The state the model is written into can be altered with the `state` parameter of the `model` attribute.
Beside `completed`, two other states can be specified:
 * `ready`
```rust
actorscript! {
    #[model(state = ready)]
    1: a[A2B] -> b
};
```
will build and check the model but running the model and  waiting for completion of the model
 is left to the user by calling
```
model.run().await?;
```
 * `running`
```rust
actorscript! {
    #[model(state = running)]
    1: a[A2B] -> b
};
```
will execute the model and waiting for completion of the model is left to the user by calling
```
model.await?;
```
  * and `completed`
```rust
actorscript! {
    #[model(state = completed)]
    1: a[A2B] -> b
};
```
will execute the model and wait for its completion.

Clients are consumed by their namesake actors and are no longer available after `actorscript`.
If access to a client is still required after `actorscript`, the token `&` can be inserted before the client e.g.
 ```rust
actorscript! {
    #[model(state = completed)]
    1: a[A2B] -> &b
};
```
Here the client `b` is wrapped into an [`Arc`]`<`[`Mutex`]`<_>>` container, cloned and passed to the associated actor.
A reference to client `b` can then be retrieved latter with:
```
let b_ref = b.lock().await.deref();
```

## Model growth

A model grows by expanding **chains** with new links and adding new **flows**.

A **chain** grows by adding new clients and ouputs e.g.
```rust
actorscript! {
    1: a[A2B] -> b[B2C] -> c
};
```
where the output `B2C` is added to `b` and connected to the client `c`.

A new **flow** is added with
```rust
actorscript! {
    1: a[A2B] -> b[B2C] -> c
    10: c[C2D] -> d
};
```
Here the new **flow**  is down sampled with a sampling rate that is 1/10th of the nominal sampling rate.

Up sampling can be obtained similarly:
```rust
actorscript! {
    1: a[A2B] -> b[B2C] -> c
    10: c[C2D] -> d
    5: d[D2E] -> e
};
```
In the model above, `C2D` is sent to `d` from `c` every 10 samples
and `D2E` is sent consecutively twice to `e` from `d` within intervals of 10 samples.

The table below gives the sampling rate for the inputs and outputs of each client:

|        | `a` | `b` | `c` | `d` | `e` |
|--------|:---:|:---:|:---:|:---:|:---:|
| inputs | 0   | 1   | 1   | 10  | 5   |
| outputs| 1   | 1   | 10  | 5   | 0   |

## Rate transitions

The former example illustrates how rate transitions can happen "naturally" between client by
relying on the up and down sampling implementations within the actors.
However, this works only if the inputs and/or outputs of a client are only used once per **flow**.

Considering the following example:
```rust
actorscript! {
    1: a[A2B] -> b[B2C] -> d
    10: c[C2D] -> b
};
```
The table of inputs and outputs sampling rate is in this case

|        | `a` | `b` | `c` | `d` |
|--------|:---:|:---:|:---:|:---:|
| inputs | 0   | 1   | 0   | 1   |
| outputs| 1   | 1   | 10  | 0   |

Here there is a mismatch between the `C2D` output with a 1/10th sampling rate
and `b` inputs that have inherited a sampling rate of 1 from the 1st **flow**.

`actorscript` is capable of detecting such mismatch, and it will introduce a rate transition client
between `c` and `b`, effectively rewriting the model as
```rust
actorscript! {
    1: a[A2B] -> b[B2C] -> d
    10: c[C2D] -> r
    1: r[C2D] -> b
};
```
where `r` is the up sampling rate transition client [Sampler].

## Feedback loop

An example of a feedback loop is a closed **chain** within a **flow** e.g.:
```rust
actorscript! {
    1: a[A2B] -> b[B2C] -> c[C2B]! -> b
};
```
The flow of data is initiated by the leftmost client (`a`)
and `b` is blocking until it receives `A2B` and `C2B` but `c` cannot send `C2B` until he has received `B2C` from `b`,
so both `b` and `c` are waiting for each other.
To break this kind of stalemate, one can instruct a client to send the data of a given output immediately by appending
the output with the token `!`.

In the above example, `c` is sending `C2B` at the same time as `a` is sending `A2B` hence allowing `b` to proceed.

Another example of a feedback loop across 2 **flows**:
```rust
actorscript! {
    1: a[A2B] -> b[B2C] -> c
    10: c[C2D]! -> d[D2B] -> b 
};
```
This version would work as well:
```rust
actorscript! {
    1: a[A2B] -> b[B2C] -> c
    10: c[C2D] -> d[D2B]! -> b 
};
```

## Output data logging

Logging the data of and output is triggered by appending the token `$` after the output like so

```rust
actorscript! {
    1: a[A2B]$ -> b[B2C]$ -> c
    10: c[C2D]$ -> d[DD]$
};
```
where `A2B` and `B2C` output data are logged into the [parquet] file `data_1.parquet` and
`C2D` and `DD` output data are logged into the [parquet] file `data_10.parquet`.
For logging purposes, `actorscript` rewrites the model as 
```rust
actorscript! {
    1: a[A2B] -> b[B2C] -> c
    10: c[C2D] -> d[DD]
    1: a[A2B] -> l1
    1: b[B2C] -> l1
    10: c[C2D] -> l10
    10: d[DD] -> l10
};
```
where `l1` and `l10` are two [Arrow] logging clients.

[gmt_dos-actors]: https://docs.rs/gmt_dos-actors
[Domain Specific Language]: https://en.wikipedia.org/wiki/Domain-specific_language
[`Write<A2B>`]: https://docs.rs/gmt_dos-clients/latest/gmt_dos_clients/interface/trait.Write.html
[`Read<A2B>`]: https://docs.rs/gmt_dos-clients/latest/gmt_dos_clients/interface/trait.Read.html
[`Arc`]: https://doc.rust-lang.org/std/sync/struct.Arc.html
[`Mutex`]: https://docs.rs/tokio/latest/tokio/sync/struct.Mutex.html#
[Sampler]: https://docs.rs/gmt_dos-clients/latest/gmt_dos_clients/struct.Sampler.html
[parquet]: https://parquet.apache.org/
[Arrow]: https://docs.rs/gmt_dos-clients_arrow/latest/gmt_dos_clients_arrow/
