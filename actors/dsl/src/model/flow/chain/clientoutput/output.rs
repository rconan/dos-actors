use std::{
    collections::{HashSet, hash_map::DefaultHasher},
    fmt::Display,
    hash::{Hash, Hasher},
};

use proc_macro2::{Span, TokenStream};
use quote::quote;
use syn::{
    Expr, Ident, Token, Type, TypePath, braced, bracketed,
    parse::{Parse, ParseBuffer, ParseStream},
    token::{Brace, Bracket},
};

use crate::client::SharedClient;

/// Actor ouput
#[derive(Debug, Clone)]
pub struct Output {
    // output type
    pub ty: Type,
    pub name: String,
    // ouput options: bootstrap, unbounded
    pub options: Option<Vec<Ident>>,
    // need a rate transition
    pub rate_transition: Option<SharedClient>,
    // need a scope
    pub scope: bool,
    pub logging: Option<Option<Expr>>,
}

impl Display for Output {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let ty = &self.ty;
        write!(f, "[{}]", quote!(#ty))
    }
}

impl Output {
    /// Creates a new output
    pub fn new(ty: Type) -> syn::Result<Self> {
        if let Type::Path(TypePath { path, .. }) = &ty {
            if let Some(ident) = path.get_ident() {
                Ok(ident.to_string().to_lowercase())
            } else {
                let mut hasher = DefaultHasher::new();
                path.hash(&mut hasher);
                Ok(format!("{}", hasher.finish()))
                /*
                                 let syn::Path { segments, .. } = path;
                segments
                    .last()
                    .map(|segment| segment.ident.to_string().to_lowercase())
                    .ok_or(syn::Error::new(
                        Span::call_site(),
                        &format!("failed to get Output ident from Type {:}", quote!(#ty)),
                    )) */
            }
        } else {
            Err(syn::Error::new(
                Span::call_site(),
                &format!("expected Output Type variant Path found {:}", quote!(#ty)),
            ))
        }
        .map(|name| Self {
            ty,
            name,
            // generics,
            options: None,
            rate_transition: None,
            scope: false,
            logging: None,
        })
    }
    pub fn expand_name(&self) -> TokenStream {
        let ty = &self.ty;
        quote!(#ty)
    }
    /// Clone and collect any sampler clients
    pub fn collect(&self, clients: &mut HashSet<SharedClient>) {
        self.rate_transition
            .as_ref()
            .map(|client| clients.insert(client.clone()));
    }
    /// Add a rate transition sampler client
    pub fn add_rate_transition(&mut self, output_rate: usize, input_rate: usize) {
        self.rate_transition = Some(SharedClient::sampler(
            self.name.as_str(),
            output_rate,
            input_rate,
        ));
    }
    pub fn add_option(&mut self, option: &str) {
        self.options
            .get_or_insert(vec![])
            .push(Ident::new(option, Span::call_site()));
    }
    pub fn add_logging(&mut self, size: Option<Expr>) {
        self.logging = Some(size);
    }
    pub fn add_scope(&mut self) {
        self.scope = true;
    }
}
impl<'a> TryFrom<ParseBuffer<'a>> for Output {
    type Error = syn::parse::Error;

    fn try_from(content: ParseBuffer<'a>) -> Result<Self, Self::Error> {
        content.parse::<Type>().and_then(|ty| Output::new(ty))
    }
}
pub struct MaybeOutput(Option<Output>);
impl From<Output> for MaybeOutput {
    fn from(value: Output) -> Self {
        Self(Some(value))
    }
}
impl MaybeOutput {
    pub fn into_inner(self) -> Option<Output> {
        self.0
    }
}
impl Parse for MaybeOutput {
    fn parse(input: ParseStream) -> syn::Result<Self> {
        // looking for an output name within brackets i.e. client[output_name]
        if input.peek(Bracket) {
            let content;
            let _ = bracketed!(content in input);
            let mut output = Output::try_from(content)?;
            // checking out for output options either !, .. or $ ,
            // or any combination of the 3 after the output i.e.
            // client[output_name]!$
            loop {
                match (
                    input.peek(Token![!]),
                    input.peek(Token![$]),
                    input.peek(Token![..]),
                    input.peek(Token![~]),
                ) {
                    (true, false, false, false) => {
                        input
                            .parse::<Token![!]>()
                            .map(|_| output.add_option("bootstrap"))?;
                    }
                    (false, true, false, false) => {
                        input.parse::<Token![$]>().map(|_| {
                            let size = if input.peek(Brace) {
                                let content;
                                let _ = braced!(content in input);
                                Some(content.parse::<Expr>()?)
                            } else {
                                None
                            };
                            output.add_logging(size);
                            Ok(())
                        })??;
                    }
                    (false, false, true, false) => {
                        input
                            .parse::<Token![..]>()
                            .map(|_| output.add_option("unbounded"))?;
                    }
                    (false, false, false, true) => {
                        input.parse::<Token![~]>().map(|_| output.add_scope())?;
                    }
                    (false, false, false, false) => break,
                    _ => panic!(
                        "only the following combination of tokens is allowed: !, $, .. and ~"
                    ),
                }
            }
            Ok(output.into())
        } else {
            Ok(MaybeOutput(None))
        }
    }
}
