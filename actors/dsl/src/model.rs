use std::{collections::HashSet, fmt::Display, sync::Arc};

use proc_macro2::Span;
use quote::quote;
use syn::{
    Attribute, Ident, LitInt, LitStr,
    parse::{Parse, ParseStream},
};

use crate::{Expand, Expanded, TryExpand, client::SharedClient};

mod flow;
use flow::Flow;

mod keyparam;
use keyparam::{KeyParam, KeyParams};

mod modelstate;
use modelstate::ModelState;

mod scope;
pub use scope::{Scope, ScopeSignal};

#[derive(Debug, Clone, Default, Hash, PartialEq, Eq)]
pub struct ModelAttributes {
    pub labels: Option<KeyParams>,
    pub images: Option<KeyParams>,
}

/**
Actors model

A model is a succession of data [Flow]s with [SharedClient]s:

Model
 |- Flow
     |- Chain
         |- ClientOuputPair
             |- SharedClient
             |- Output
 |- SharedClient
*/
#[derive(Debug, Clone, Default)]
pub(super) struct Model {
    pub name: Option<Ident>,
    pub state: ModelState,
    pub flowchart: Option<LitStr>,
    pub clients: HashSet<SharedClient>,
    pub flows: Vec<Flow>,
    pub scope: Scope,
    pub attributes: Arc<ModelAttributes>,
}

impl Display for Model {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        writeln!(
            f,
            "Model {} [{}] :",
            self.name().to_string().to_uppercase(),
            self.state
        )?;
        for flow in self.flows.iter() {
            writeln!(f, "{flow}")?;
        }
        Ok(())
    }
}

impl Model {
    /// Returns the model name
    pub fn name(&self) -> Ident {
        self.name
            .clone()
            .unwrap_or_else(|| Ident::new("model", Span::call_site()))
    }
    /// Parse model attributes
    ///
    /// #[model(key = param,...)]
    pub fn attributes(mut self, attrs: Option<Vec<Attribute>>) -> syn::Result<Self> {
        let Some(attrs) = attrs else { return Ok(self) };
        let mut model_attributes = ModelAttributes::default();
        for attr in attrs {
            match &attr
                .path()
                .get_ident()
                .map(|id| id.to_string())
                .as_ref()
                .map(|s| s.as_str())
            {
                Some("model") => {
                    for kp in attr.parse_args::<KeyParams>()? {
                        let KeyParam { key, param, .. } = kp;
                        match key.to_string().as_str() {
                            "name" => {
                                self.name = Ident::try_from(&param).ok();
                            }
                            "state" => {
                                self.state = ModelState::try_from(&param)?;
                            }
                            "flowchart" => {
                                self.flowchart = LitStr::try_from(&param).ok();
                            }
                            _ => {
                                return Err(syn::Error::new(
                                    Span::call_site(),
                                    r#"expected model attributes "name" or "state", found {key}"#,
                                ));
                            }
                        }
                    }
                }
                Some("labels") => {
                    model_attributes.labels = Some(attr.parse_args::<KeyParams>()?);
                }
                Some("images") => {
                    model_attributes.images = Some(attr.parse_args::<KeyParams>()?);
                }
                Some("scope") => (),
                Some(value) => {
                    panic!("found model attribute: {value}, expected model, labels or images")
                }
                None => panic!("expected Some model attribute, found None"),
            }
        }
        self.attributes = Arc::new(model_attributes);
        Ok(self)
    }
    /// Build the model
    pub fn build(mut self) -> Self {
        let name = self.name();
        let mut flow_implicits: Vec<_> = self
            .flows
            .iter()
            .flat_map(|flow| flow.implicits(&name, &mut self.scope))
            .collect();
        self.flows.append(&mut flow_implicits);

        self.flows
            .iter_mut()
            .for_each(|flow| flow.dedup(&mut self.clients));

        self.flows.iter_mut().for_each(|flow| flow.match_rates());

        self.flows
            .iter()
            .for_each(|flow| flow.collect_clients(&mut self.clients));

        self
    }
}

impl Parse for Model {
    fn parse(input: ParseStream) -> syn::Result<Self> {
        let mut flows: Vec<Flow> = vec![];
        while input.peek(LitInt) {
            flows.push(input.parse::<Flow>()?);
        }

        Ok(Self {
            clients: HashSet::new(),
            flows,
            ..Default::default()
        })
    }
}

impl TryExpand for Model {
    fn try_expand(&self) -> syn::Result<Expanded> {
        let mut client_defs = vec![];
        let mut actors = vec![];
        let mut actor_defs = vec![];
        for client in self.clients.iter() {
            let q = client.expand();
            if client_defs
                .iter()
                .find(|s: &&proc_macro2::TokenStream| s.to_string() == q.to_string())
                .is_none()
            {
                client_defs.push(q);
                actors.push(client.actor());
                actor_defs.push(client.borrow().actor_declaration());
            }
        }
        let labels = self.attributes.labels.as_ref().map(|labels| {
            labels
                .iter()
                .map(|KeyParam { key, param, .. }| {
                    let p = param.expand();
                    quote!(
                        #key.set_label(#p);
                    )
                })
                .collect::<Vec<_>>()
        });
        let images = self.attributes.images.as_ref().map(|images| {
            images
                .iter()
                .map(|KeyParam { key, param, .. }| {
                    let p = param.expand();
                    quote!(
                        #key.set_image(#p);
                    )
                })
                .collect::<Vec<_>>()
        });
        let flows: Vec<_> = self.flows.iter().map(|flow| flow.expand()).collect();
        let (model, name) = match (self.name.clone(), self.flowchart.clone()) {
            (None, None) => {
                let model = Ident::new("model", Span::call_site());
                let name = LitStr::new("model", Span::call_site());
                (model, name)
            }
            (None, Some(name)) => {
                let model = Ident::new("model", Span::call_site());
                (model, name)
            }
            (Some(model), None) => {
                let name = LitStr::new(&model.to_string(), model.span());
                (model, name)
            }
            (Some(model), Some(name)) => (model, name),
        };
        let state = match self.state {
            ModelState::Ready => quote!(check()?),
            ModelState::Running => quote!(check()?.run()),
            ModelState::Completed => quote!(check()?.run().await?),
        };
        // println!("{state}");
        let code = match (labels, images) {
            (Some(labels), Some(images)) => {
                quote! {
                    // ACTORS DEFINITION
                    #(#client_defs)*
                    #(#labels)*
                    #(#images)*
                    #(#actor_defs)*
                    // FLOWS DEFINITION
                    #(#flows)*
                }
            }
            (Some(labels), None) => {
                quote! {
                    // ACTORS DEFINITION
                    #(#client_defs)*
                    #(#labels)*
                    #(#actor_defs)*
                    // FLOWS DEFINITION
                    #(#flows)*
                }
            }
            (None, Some(images)) => {
                quote! {
                    // ACTORS DEFINITION
                    #(#client_defs)*
                    #(#images)*
                    #(#actor_defs)*
                    // FLOWS DEFINITION
                    #(#flows)*
                }
            }
            (None, None) => {
                quote! {
                    // ACTORS DEFINITION
                    #(#client_defs)*
                    #(#actor_defs)*
                    // FLOWS DEFINITION
                    #(#flows)*
                }
            }
        };
        Ok(
            if let Some(_) = self.clients.iter().find(|client| client.is_scope()) {
                let scope_client = self.scope.try_expand()?;
                quote! {
                    let mut monitor = ::gmt_dos_clients_scope::server::Monitor::new();
                    #code
                    // MODEL
                    #[allow(unused_variables)]
                    let #model = ::gmt_dos_actors::prelude::model!(#(#actors),*).name(#name);
                    // .flowchart()
                    let #model = ::gmt_dos_actors::prelude::FlowChart::flowchart_open(#model).check()?.run();
                    #scope_client
                    #model.await?;
                    monitor.await?;
                }
            } else {
                quote! {
                    #code
                    // MODEL
                    #[allow(unused_variables)]
                    let #model = ::gmt_dos_actors::prelude::model!(#(#actors),*).name(#name);
                    // .flowchart()
                    // let #model = ::gmt_dos_actors::ramework::model::FlowChart(#model);
                    let model = ::gmt_dos_actors::prelude::FlowChart::flowchart_open(#model).#state;
                }
            },
        )
    }
}
