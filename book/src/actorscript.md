# actorscript

`actorscript` is a macro that interprets a script that describes a`gmt_dos-actors` model.
The macro generates all the boilerplate code to build actors and to link the inputs and the outputs of actors.

The actorscript [documentation](https://docs.rs/gmt_dos-actors_dsl) shows how to build a model with this domain specific language (DSL). 
The language syntax is given [here](https://docs.rs/gmt_dos-actors_dsl/latest/gmt_dos_actors_dsl/macro.actorscript.html)

As an example, the feedback model is rewritten with `actorscript`
```rust,no_run,noplayground
{{#include ../examples/feedback-dsl.rs:actorscript}}
```
