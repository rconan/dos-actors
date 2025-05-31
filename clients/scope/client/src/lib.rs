/*!
# Scope client (`default` feature)

The scopes defined in the client module receive data from the scope servers
and show a live display of the data.

See also [gmt_dos_clients_scope].

## Example

```ignore
use gmt_dos_clients_scope::client;

#[derive(gmt_dos_clients::interface::UID)]
#[uid(port = 5001)]
pub enum Signal {}


gmt_dos_clients_scope::client::Scope::new()
    .signal::<Signal>().unwrap()
    .show();
```

*/

pub use gmt_dos_clients_scope::{
    GmtScope, ImageScope, ImageScopeKind, PlotScope, ScopeKind,
    client::{ClientError, GmtShot, GridScope, Scope, Shot, XScope},
};
