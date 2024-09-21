use std::{fmt::Display, hash::Hash};

use proc_macro2::{Literal, Span};
use quote::quote;
use syn::{parse::Parse, Expr, Ident, LitInt, LitStr, Token};

use crate::{model::ScopeSignal, Expand, Expanded};

const LOG_BUFFER_SIZE: usize = 1_000;

#[derive(Debug, Clone, Hash, PartialEq, Eq)]
pub struct System {
    pub name: Ident,
    pub io: Option<Ident>,
}

impl Parse for System {
    fn parse(input: syn::parse::ParseStream) -> syn::Result<Self> {
        Ok(Self {
            name: input.parse()?,
            io: input.parse::<Token![::]>().and_then(|_| input.parse()).ok(),
        })
    }
}

#[derive(Debug, Clone, Eq, PartialEq, Hash)]
pub enum ClientKind {
    MainScope,
    Sampler,
    Logger(Ident, Option<Expr>),
    Scope { signal: ScopeSignal },
    SubSystem(System),
}
impl ClientKind {
    pub fn is_scope(&self) -> bool {
        match self {
            Self::Scope { .. } => true,
            _ => false,
        }
    }
}

/// Actor client
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub struct Client {
    // client variable name
    pub name: Ident,
    // actor variable name
    pub actor: Ident,
    // actor inputs rate
    pub input_rate: usize,
    // actor output rates
    pub output_rate: usize,
    // client type
    pub kind: ClientKind,
}
impl Client {
    pub fn lit_output_rate(&self) -> LitInt {
        Literal::usize_unsuffixed(self.output_rate).into()
    }
    pub fn lit_input_rate(&self) -> LitInt {
        Literal::usize_unsuffixed(self.input_rate).into()
    }
    pub fn into_input(&self) -> Expanded {
        let actor = &self.actor;
        match &self.kind {
            ClientKind::Logger(_, None) => quote!(
                // .log(&mut #actor).await?;
                gmt_dos_actors::framework::network::IntoLogs::log(output, &mut #actor).await?;
            ),
            ClientKind::Logger(_, Some(size)) => quote!(
                // .logn(&mut #actor, #size).await?;
                gmt_dos_actors::framework::network::IntoLogsN::logn(output, &mut #actor, #size).await?;
            ),
            ClientKind::SubSystem(System { io: Some(io), .. }) => quote!(
                // .into_input(&mut #actor)?;
                gmt_dos_actors::framework::network::TryIntoInputs::into_input::<#io>(output, &mut #actor)?;
            ),
            _ => quote!(
                // .into_input(&mut #actor)?;
                gmt_dos_actors::framework::network::TryIntoInputs::into_input(output, &mut #actor)?;
            ),
        }
    }
    pub fn is_scope(&self) -> bool {
        self.kind.is_scope()
    }
    pub fn actor_declaration(&self) -> Expanded {
        match self.kind {
            ClientKind::SubSystem(_) => quote!(),
            _ => {
                let Self { name, actor, .. } = self;
                let (i, o) = (self.lit_input_rate(), self.lit_output_rate());
                quote!(
                    let mut #actor: ::gmt_dos_actors::prelude::Actor<_,#i,#o> =
                        ::gmt_dos_actors::prelude::Actor::from(&#name);
                )
            }
        }
    }
}
impl Display for Client {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self.kind {
            ClientKind::MainScope | ClientKind::SubSystem(_) => write!(
                f,
                "main client: {} into actor: {} with rates: {} input & {} output",
                self.name, self.actor, self.input_rate, self.output_rate
            ),
            ClientKind::Sampler => write!(
                f,
                "Sampler client: {} into actor: {} with rates: {} input & {} output",
                self.name, self.actor, self.input_rate, self.output_rate
            ),
            ClientKind::Logger(..) => write!(
                f,
                "Arrow client: {} into actor: {} with rates: {} input & {} output",
                self.name, self.actor, self.input_rate, self.output_rate
            ),
            ClientKind::Scope { .. } => write!(
                f,
                "Scope client: {} into actor: {} with rates: {} input & {} output",
                self.name, self.actor, self.input_rate, self.output_rate
            ),
        }
    }
}
impl Expand for Client {
    fn expand(&self) -> Expanded {
        let Self {
            name, actor, kind, ..
        } = self;
        let (i, o) = (self.lit_input_rate(), self.lit_output_rate());
        match kind {
            ClientKind::MainScope => {
                quote! {
                    let mut #name = ::gmt_dos_actors::client::Client::from(#name);
                    // let mut #actor: ::gmt_dos_actors::prelude::Actor<_,#i,#o> =
                    //     ::gmt_dos_actors::prelude::Actor::from(&#name);
                }
            }
            ClientKind::SubSystem(_) => quote!(
                let mut #actor = #name.clone();
            ),
            ClientKind::Sampler => {
                let sampler_type = LitStr::new(
                    if self.input_rate < self.output_rate {
                        "downsampling"
                    } else {
                        "upsampling"
                    },
                    Span::call_site(),
                );
                quote! {
                    let mut #name = ::gmt_dos_actors::client::Client::from(::gmt_dos_clients::Sampler::default());
                    #name.set_label(format!("{}\n{}:{}",#sampler_type,#i,#o));
                }
            }
            ClientKind::Logger(model_name, _) => {
                let filename = LitStr::new(&format!("{actor}"), Span::call_site());
                let buffer_size = LitInt::new(&format!("{LOG_BUFFER_SIZE}"), Span::call_site());
                quote! {
                    let mut #name = ::gmt_dos_actors::client::Client::from(
                        ::gmt_dos_clients_arrow::Arrow::builder(#buffer_size)
                        .filename(#filename)
                        .build());
                    #name.set_label(#filename);
                }
            }
            ClientKind::Scope {
                signal: ScopeSignal { ty, .. },
            } => {
                quote! {
                    let mut #name = ::gmt_dos_actors::client::Client::from(
                        ::gmt_dos_clients_scope::server::Scope::<#ty>::builder(&mut monitor)
                            .build()?);
                }
            }
        }
    }
}
