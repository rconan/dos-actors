use std::{collections::VecDeque, ops::Deref};

use proc_macro2::Span;
use quote::quote;
use syn::{
    Ident, LitStr, Token,
    parse::{Parse, ParseStream},
};

use crate::Expand;

/// Parameter type
#[derive(Debug, Clone, Hash, PartialEq, Eq)]
pub enum Param {
    Ident(Ident),
    LitStr(LitStr),
}

impl Expand for Param {
    fn expand(&self) -> crate::Expanded {
        match self {
            Param::Ident(value) => quote!(#value),
            Param::LitStr(value) => quote!(#value),
        }
    }
}

impl TryFrom<&Param> for Ident {
    type Error = syn::Error;

    fn try_from(value: &Param) -> Result<Self, Self::Error> {
        match value {
            Param::Ident(value) => Ok(value.clone()),
            Param::LitStr(_) => Err(syn::Error::new(
                Span::call_site(),
                "expect Ident parameter, found LitStr",
            )),
        }
    }
}
impl TryFrom<&Param> for LitStr {
    type Error = syn::Error;

    fn try_from(value: &Param) -> Result<Self, Self::Error> {
        match value {
            Param::LitStr(value) => Ok(value.clone()),
            Param::Ident(_) => Err(syn::Error::new(
                Span::call_site(),
                "expect LitStr parameter, found Ident",
            )),
        }
    }
}

impl Parse for Param {
    fn parse(input: ParseStream) -> syn::Result<Self> {
        input.parse::<Ident>().map_or_else(
            |_e| input.parse::<LitStr>().map(|value| Self::LitStr(value)),
            |value| Ok(Self::Ident(value)),
        )
    }
}

/// A key/parameter pair
///
/// Parsed as key=parameter
#[derive(Debug, Clone, Hash, PartialEq, Eq)]
pub struct KeyParam {
    pub key: Ident,
    pub param: Param,
}

impl Parse for KeyParam {
    fn parse(input: ParseStream) -> syn::Result<Self> {
        let key: Ident = input.parse()?;
        let _ = input.parse::<Token!(=)>()?;
        let param: Param = input.parse()?;
        Ok(Self { key, param })
    }
}

/// A collection of key/parameter pairs
///
/// Parsed as key1=parameter1, key2=parameter2, ...
#[derive(Debug, Clone, Hash, PartialEq, Eq)]
pub struct KeyParams(VecDeque<KeyParam>);

impl Deref for KeyParams {
    type Target = VecDeque<KeyParam>;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl Iterator for KeyParams {
    type Item = KeyParam;

    fn next(&mut self) -> Option<Self::Item> {
        self.0.pop_front()
    }
}

impl Parse for KeyParams {
    fn parse(input: ParseStream) -> syn::Result<Self> {
        Ok(Self(
            input
                .parse_terminated(KeyParam::parse, Token!(,))?
                .into_iter()
                .collect(),
        ))
    }
}
