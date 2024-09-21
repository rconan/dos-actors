use std::{cell::RefCell, fmt::Display, hash::Hash, ops::Deref, rc::Rc};

use proc_macro2::Span;
use quote::format_ident;
use syn::{Expr, Ident, Type};

use crate::{
    model::{Scope, ScopeSignal},
    Expand, Expanded,
};

use super::{Client, ClientKind, System};

/// Shared client with interior mutability
#[derive(Debug, Clone, Eq)]
pub struct SharedClient(Rc<RefCell<Client>>);
impl SharedClient {
    /// Creates a new client from the main scope
    pub fn new(name: Ident) -> Self {
        /*         let actor = if let Reference::Value = reference {
            name.clone()
        } else {
            Ident::new(&format!("{name}_actor"), Span::call_site())
        }; */
        let actor = Ident::new(&format!("{name}_actor"), Span::call_site());
        Self(Rc::new(RefCell::new(Client {
            name,
            actor,
            input_rate: 0,
            output_rate: 0,
            kind: ClientKind::MainScope,
        })))
    }
    /// Creates a new subsystem actor
    pub fn subsystem(sys: System) -> Self {
        let name = sys.name.clone();
        let actor = Ident::new(&format!("{name}_clone"), Span::call_site());
        Self(Rc::new(RefCell::new(Client {
            name,
            actor,
            input_rate: 0,
            output_rate: 0,
            kind: ClientKind::SubSystem(sys),
        })))
    }
    /// Creates a sampler client from [gmt_dos-clients::Sampler](https://docs.rs/gmt_dos-clients/latest/gmt_dos_clients/struct.Sampler.html)
    pub fn sampler(name: &str, output_rate: usize, input_rate: usize) -> Self {
        let sampler = format_ident!("_{}_{}_{}_", input_rate, name, output_rate);
        Self(Rc::new(RefCell::new(Client {
            name: sampler.clone(),
            actor: sampler,
            input_rate,
            output_rate,
            kind: ClientKind::Sampler,
        })))
    }
    /// Creates a sampler client from [gmt_dos-clients_arrow](https://docs.rs/gmt_dos-clients_arrow)
    pub fn logger(model_name: &Ident, input_rate: usize, size: Option<Expr>) -> Self {
        let name = format_ident!("{}__logging_{}", model_name, input_rate);
        let actor = format_ident!("{}__data_{}", model_name, input_rate);
        Self(Rc::new(RefCell::new(Client {
            name,
            actor,
            input_rate,
            output_rate: 0,
            kind: ClientKind::Logger(model_name.clone(), size),
        })))
    }
    /// Creates a scope client from [gmt_dos-clients_scope](https://docs.rs/gmt_dos-clients_scope)
    pub fn scope(
        output_type: &Type,
        output_name: &str,
        input_rate: usize,
        scope: &mut Scope,
    ) -> Self {
        // let name = Ident::new(&format!("scope_{}", output_name), output_name.span());
        let scope_signal = ScopeSignal {
            ty: output_type.clone(),
            name: output_name.to_string(),
        };
        scope.signals.push(scope_signal.clone());
        let actor = format_ident!("scope_{}", output_name);
        Self(Rc::new(RefCell::new(Client {
            name: actor.clone(),
            actor,
            input_rate,
            output_rate: 0,
            kind: ClientKind::Scope {
                signal: scope_signal,
            },
        })))
    }
    // pub fn name(&self) -> Ident {
    //     self.0.borrow().name.clone()
    // }
    pub fn actor(&self) -> Ident {
        self.borrow().actor.clone()
    }
    pub fn is_scope(&self) -> bool {
        self.borrow().is_scope()
    }
}
impl Display for SharedClient {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        self.borrow().fmt(f)
    }
}
impl Expand for SharedClient {
    fn expand(&self) -> Expanded {
        self.borrow().expand()
    }
}
impl Deref for SharedClient {
    type Target = RefCell<Client>;

    fn deref(&self) -> &Self::Target {
        self.0.deref()
    }
}

impl Hash for SharedClient {
    fn hash<H: std::hash::Hasher>(&self, state: &mut H) {
        self.borrow().hash(state);
    }
}
impl PartialEq for SharedClient {
    fn eq(&self, other: &Self) -> bool {
        self.0 == other.0
    }
}

impl From<Client> for SharedClient {
    fn from(client: Client) -> Self {
        Self(Rc::new(RefCell::new(client)))
    }
}
