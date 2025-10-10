use std::{
    fmt::{self, Display},
    ops::Deref,
};

/// FEM inputs/ouputs names & descriptions
#[derive(Default)]
pub struct Name {
    pub name: String,
    pub description: Vec<String>,
}
impl fmt::Debug for Name {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Name")
            .field("name", &self.name)
            .field("description", &self.description[0])
            .finish()
    }
}
impl Deref for Name {
    type Target = str;

    fn deref(&self) -> &Self::Target {
        self.name.as_str()
    }
}
impl Display for Name {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self.name)
    }
}
impl<S: Into<String>> From<S> for Name {
    fn from(name: S) -> Self {
        Name {
            name: name.into(),
            ..Default::default()
        }
    }
}
impl From<&Name> for String {
    fn from(value: &Name) -> Self {
        value.name.clone()
    }
}
impl Name {
    /// Adds to the [Name] description
    pub fn push_description(&mut self, description: String) {
        self.description.push(description);
    }
    /// Returns the input or output `name` as `Enum` variant type
    pub fn variant(&self) -> String {
        self.split("_")
            .map(|s| {
                let (first, last) = s.split_at(1);
                first.to_uppercase() + last
            })
            .collect::<String>()
    }
    /**
    Returns the code representing the input or ouput as an empty `Enum`

    ```ignore
    pub enum {variant} {}
    ```
    */
    pub fn enum_variant(&self) -> String {
        let descriptions: Vec<_> = self
            .description
            .iter()
            .map(|d| {
                format!(
                    r##"
 1. {}
            "##,
                    d
                )
            })
            .collect();
        format!(
            r##"
            #[doc = "{name}"]
            #[doc = ""]
            #[doc = "{descriptions}"]
        #[derive(Debug, ::interface::UID)]
        pub enum {variant} {{}}
        "##,
            name = self.name,
            descriptions = descriptions.join("\n"),
            variant = self.variant()
        )
    }
    /**
    Returns the code implementing `FemIo<variant>`

    ```ignore
        impl FemIo<{variant}> for Vec<Option<{io}>> {
            fn position(&self) -> Option<usize>{
                self.iter().filter_map(|x| x.as_ref())
                        .position(|x| if let {io}::{variant}(_) = x {true} else {false})
            }
        }
    ```
    where `io` is another `Enum` that may have the same `variant`
    */
    pub fn impl_enum_variant_for_io(&self, io: &str) -> String {
        format!(
            r##"
        impl FemIo<{variant}> for Vec<Option<{io}>> {{
            fn position(&self) -> Option<usize>{{
                self.iter().filter_map(|x| x.as_ref())
                        .position(|x| if let {io}::{variant}(_) = x {{true}} else {{false}})
            }}
        }}
        "##,
            variant = self.variant(),
            io = io
        )
    }
}

/// A list of FEM inputs or outputs [Name]
#[derive(Debug, Default)]
pub struct Names(Vec<Name>);
impl Names {
    /// Searches for a particular name
    ///
    /// Return `Some(name)` if it exists
    pub fn find<S: AsRef<str>>(&self, aname: S) -> Option<String> {
        self.iter()
            .find(|name| name.variant().as_str() == aname.as_ref())
            .map(|name| name.name.clone())
    }
}
impl FromIterator<Name> for Names {
    fn from_iter<T: IntoIterator<Item = Name>>(iter: T) -> Self {
        Self(iter.into_iter().collect())
    }
}
impl FromIterator<String> for Names {
    fn from_iter<T: IntoIterator<Item = String>>(iter: T) -> Self {
        Self(iter.into_iter().map(|x| x.into()).collect())
    }
}
impl Deref for Names {
    type Target = Vec<Name>;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}
impl Display for Names {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        for variant in self.iter() {
            write!(f, "{}", variant.enum_variant())?;
        }
        Ok(())
    }
}
