use std::{collections::HashSet, fmt::Display};

use syn::{
    LitInt, Token,
    parse::{Parse, ParseStream},
};

use crate::{Expand, Expanded, client::SharedClient};

mod chain;
use chain::Chain;

use super::Scope;

/// Data flow
///
/// A flow is characterized by a sampling rate and
/// a chain of actors i.e
///
/// rate: actor1[output1_of_actor1] -> actor2[output1_of_actor2] -> actor3
#[derive(Debug, Clone, Default)]
pub struct Flow {
    pub rate: usize,
    pub chain: Chain,
}

impl Display for Flow {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{:3}: {}", self.rate, self.chain)
    }
}

impl Flow {
    /// Iteration through the actors chain of each flow replacing duplicated client with shared reference
    pub fn dedup(&mut self, clients: &mut HashSet<SharedClient>) {
        self.chain.dedup(clients);
    }
    /// Iteration through the actors chain of each flow matching output/input rate or setting up a rate transition
    pub fn match_rates(&mut self) {
        self.chain.match_rates(self.rate);
    }
    /// Collect output special clients
    ///
    /// Special clients are [Sampler] and [Arrow]
    pub fn collect_clients(&self, clients: &mut HashSet<SharedClient>) {
        self.chain.iter().for_each(|client_output| {
            client_output
                .output
                .as_ref()
                .map(|output| output.collect(clients));
        });
    }
    /// Check for loggers & scopes
    pub fn implicits(&self, name: &syn::Ident, scope: &mut Scope) -> Vec<Flow> {
        self.chain
            .implicits(self.rate, name, scope)
            .into_iter()
            .map(|chain| Flow {
                rate: self.rate,
                chain,
            })
            .collect()
    }
}

impl Parse for Flow {
    fn parse(input: ParseStream) -> syn::Result<Self> {
        let rate = input.parse::<LitInt>()?.base10_parse::<usize>()?;
        let _: Token!(:) = input.parse()?;
        let chain = input.parse::<Chain>()?; //.logging(rate);
        Ok(Self {
            rate,
            chain,
            ..Default::default()
        })
    }
}

impl Expand for Flow {
    fn expand(&self) -> Expanded {
        self.chain.expand()
    }
}
