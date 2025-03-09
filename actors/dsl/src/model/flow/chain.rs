use std::{
    collections::HashSet,
    fmt::Display,
    ops::{Deref, DerefMut},
};

use proc_macro2::Span;
use quote::quote;
use syn::{
    parse::{Parse, ParseStream},
    punctuated::Punctuated,
    Ident, Token,
};

use crate::{
    client::{Client, ClientKind, SharedClient},
    model::Scope,
    Expand, Expanded, TryExpand,
};

pub mod clientoutput;
use clientoutput::ClientOutputPair;

use self::clientoutput::Output;

/// Chain of actors
///
/// A chain consists in pairs of a client and one output
/// A logger may be assigned to a chain if some outputs require to be logged
#[derive(Debug, Clone, Default)]
pub struct Chain {
    pub clientoutput_pairs: Vec<ClientOutputPair>,
}

impl Display for Chain {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "{}",
            self.clientoutput_pairs
                .iter()
                .map(|clientoutput| clientoutput.to_string())
                .collect::<Vec<String>>()
                .join(" -> ")
        )
    }
}

impl From<Vec<ClientOutputPair>> for Chain {
    fn from(clientoutput_pairs: Vec<ClientOutputPair>) -> Self {
        Self { clientoutput_pairs }
    }
}

impl Deref for Chain {
    type Target = Vec<ClientOutputPair>;

    fn deref(&self) -> &Self::Target {
        &self.clientoutput_pairs
    }
}
impl DerefMut for Chain {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.clientoutput_pairs
    }
}

impl Parse for Chain {
    fn parse(input: ParseStream) -> syn::Result<Self> {
        Ok(Self {
            clientoutput_pairs:
                Punctuated::<ClientOutputPair, Token![->]>::parse_separated_nonempty(input)?
                    .into_iter()
                    .collect(),
            ..Default::default()
        })
    }
}

impl Chain {
    /// Iteration through the actors chain of each flow replacing duplicated client with shared reference
    pub fn dedup(&mut self, clients: &mut HashSet<SharedClient>) {
        self.iter_mut().for_each(|client_output| {
            let client = &mut client_output.client;
            if !clients.insert(client.clone()) {
                *client = clients.get(client).unwrap().clone();
            }
        });
    }
    /// Iteration through the actors chain of each flow matching output/input rate or setting up a rate transition
    pub fn match_rates(&mut self, flow_rate: usize) {
        let mut iter = self.iter_mut().peekable();
        loop {
            match iter.next() {
                Some(ClientOutputPair {
                    client: output_client,
                    output: Some(output),
                }) => {
                    if let Some(ClientOutputPair {
                        client: input_client,
                        ..
                    }) = iter.peek_mut()
                    {
                        // a client with an output and followed by another client: output_client[output] -> input_client
                            let (output_rate, input_rate) = (
                                &mut output_client.borrow_mut().output_rate,
                                &mut input_client.borrow_mut().input_rate,
                            );
                            if *output_rate == 0 {
                                *output_rate = flow_rate;
                            }
                            if *input_rate == 0 {
                                *input_rate = flow_rate;
                            }
                            if *output_rate != *input_rate {
                                output.add_rate_transition(*input_rate, *output_rate);
                            }
                    } else {
                        // a client with an output and not followed by another client: output_client[output]
                        let output_rate = &mut output_client.borrow_mut().output_rate;
                        if *output_rate == 0 {
                            *output_rate = flow_rate;
                        }
                    }
                    // Sub-system actors always takes inputs
                    if let Client {
                        input_rate,
                        kind: ClientKind::SubSystem(..),
                        ..
                    } = &mut *output_client.borrow_mut()
                    {
                        if *input_rate == 0 {
                            *input_rate = flow_rate;
                        }
                    };
                }
                Some(ClientOutputPair {
                    client: output_client,
                    output: None,
                }) => {
                    // juts a client: output_client
                    if output_client.borrow_mut().input_rate == 0 {
                        output_client.borrow_mut().input_rate = flow_rate
                    };
                }
                None => break,
            }
        }
    }
    /// Check for loggers & scopes
    pub fn implicits(
        &self,
        rate: usize,
        model_name: &Ident,
        model_scope: &mut Scope,
    ) -> Vec<Chain> {
        self.iter()
            .filter_map(move |client_ouput| match client_ouput {
                ClientOutputPair {
                    client,
                    output:
                        Some(Output {
                            ty,
                            name,
                            options: output_options,
                            scope,
                            logging,
                            ..
                        }),
                } if *scope == true || logging.is_some() => {
                    let mut options = output_options.clone();
                    options
                        .get_or_insert(vec![])
                        .push(Ident::new("unbounded", Span::call_site()));
                    options.as_mut().map(|options| options.dedup());

                    let output = Output {
                        ty: ty.clone(),
                        name: name.clone(),
                        options,
                        rate_transition: None,
                        scope: false,
                        logging: None,
                    };

                    let left = ClientOutputPair {
                        client: client.clone(),
                        output: Some(output),
                    };

                    let mut chains = Option::<Vec<Chain>>::None;
                    if *scope {
                        chains.get_or_insert(vec![]).push(
                            vec![
                                left.clone(),
                                SharedClient::scope(&ty, &name, rate, model_scope).into(),
                            ]
                            .into(),
                        )
                    }
                    if let Some(size) = logging {
                        chains.get_or_insert(vec![]).push(
                            vec![
                                left.clone(),
                                SharedClient::logger(&model_name, rate, size.clone()).into(),
                            ]
                            .into(),
                        )
                    }
                    chains
                }
                _ => None,
            })
            .flatten()
            .collect()
    }
}

impl Expand for Chain {
    fn expand(&self) -> Expanded {
        let iter = self
            .iter()
            .skip(1)
            .map(|client_output| client_output.client.borrow().into_input());
        let outputs: Vec<_> = self
            .iter()
            .zip(iter)
            .filter_map(|(output, into_input)| {
                output.try_expand().ok().map(|add_output| {
                    quote! {
                        #add_output
                        #into_input
                    }
                })
            })
            .collect();
        quote!(#(#outputs)*)
    }
}
