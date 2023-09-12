/*!
# actorscript

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

The [gmt_dos-actors] model is written in the `ready` state meaning that in order to run the model to completion
the following line of code is needed after `actorscript`
```rust
model.run().await?;
```
The state the model is written into can be altered with the `state` parameter of the `model` attribute.
Beside `ready`, two other states can be specified:
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
let b_ref = *b.lock().await;
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
    1: a[A2B] -> logging_1
    1: b[B2C] -> logging_1
    10: c[C2D] -> logging_10
    10: d[DD] -> logging_10
};
```
where `logging_1` and `logging_10` are two [Arrow] logging clients.
References to both clients is available after `actorscript` with
```
*logging_1.lock().await
```
and
```
*logging_10.lock().await
```

[gmt_dos-actors]: https://docs.rs/gmt_dos-actors
[Domain Specific Language]: https://en.wikipedia.org/wiki/Domain-specific_language
[`Write<A2B>`]: https://docs.rs/gmt_dos-clients/latest/gmt_dos_clients/interface/trait.Write.html
[`Read<A2B>`]: https://docs.rs/gmt_dos-clients/latest/gmt_dos_clients/interface/trait.Read.html
[`Arc`]: https://doc.rust-lang.org/std/sync/struct.Arc.html
[`Mutex`]: https://docs.rs/tokio/latest/tokio/sync/struct.Mutex.html#
[Sampler]: https://docs.rs/gmt_dos-clients/latest/gmt_dos_clients/struct.Sampler.html
[parquet]: https://parquet.apache.org/
[Arrow]: https://docs.rs/gmt_dos-clients_arrow/
*/

use proc_macro::TokenStream;
use quote::quote;
use syn::{
    parse::{Parse, ParseStream},
    parse_macro_input, Attribute,
};

/**
Interpreter for the scripting language of [gmt_dos-actors] models.

Generates all the boilerplate code to build [gmt_dos-actors] models.

See also the [crate](self) documentation for details about building [gmt_dos-actors] models with [actorscript](actorscript!).

## Syntax

### Flow

```rust
1: ...
```

A **flow** always starts with an integer literal following by colon, then a **chain**.

### Chain

```rust
pair|client -> another_pair -> ... -> pair|client
```

The start and end of a chain can be either a client or a pair of a client and an ouput.

### Client-Output Pair

The syntax for a client-output pair is (optional parameters are preceded by `?`)
```rust
?prefix client ?(label) [Output] ?suffix
```

* `client`: is the name of the client identifier that is the variable declared in the main scope.
* `Output`: is the type of one of the outputs of the actor associated with the client,
the client must implement the trait `Write<Output>`,
if it preceded by another client-output pair it must also implement the `Read<PreviousOutput>` trait.
* `?prefix`: optional operator applied to the client:
  * `&`: uses a reference to the client instead of consuming it
* `?suffix`: optional operators applied to the ouput (suffix can be combined in any order (e.g `S!..` or `!..$` are both valid)):
  * `!`: output bootstrapping
  * `$`: data logging: creates clients variables `logging_<flow rate>` and data file `data_<flow rate>.parquet`,
  * `..`: unbounded output
  * `~`: stream the output to a [gmt_dos-clients_scope] client
* `label`: string litteral label given to the client actor in the flow chart (default: "client_type")

### Attributes

#### `model`

```rust
#[model(key = param, ...)]
```
Possible keys:
 * `name`: model variable identifier (default: `model`), this is also the name given to the flowchart
 * `state`: model state identifier: `ready`, `running` or `completed` (default: `ready`)
 * `flowchart`: flowchart string literal name (default `"model"`)

[gmt_dos-actors]: https://docs.rs/gmt_dos-actors
[gmt_dos-clients_scope]: https://docs.rs/gmt_dos-clients_scope
*/
#[proc_macro]
pub fn actorscript(input: TokenStream) -> TokenStream {
    let script = parse_macro_input!(input as Script);
    script
        .try_expand()
        .unwrap_or_else(syn::Error::into_compile_error)
        .into()
}

mod model;
use model::Model;
mod client;

pub(crate) type Expanded = proc_macro2::TokenStream;

/// Source code expansion
pub(crate) trait Expand {
    fn expand(&self) -> Expanded;
}
/// Faillible source code expansion
pub(crate) trait TryExpand {
    fn try_expand(&self) -> syn::Result<Expanded>;
}

/// Script parser
///
/// The script parser holds the code of the actors model
#[derive(Debug, Clone)]
struct Script {
    model: Model,
}

impl Parse for Script {
    fn parse(input: ParseStream) -> syn::Result<Self> {
        let attrs = input.call(Attribute::parse_outer).ok();
        let model = input.parse::<Model>()?.attributes(attrs)?.build();
        println!("/*\n{model} */");
        Ok(Script { model })
    }
}

impl TryExpand for Script {
    fn try_expand(&self) -> syn::Result<Expanded> {
        let model = self.model.try_expand()?;
        Ok(quote!(#model))
    }
}
