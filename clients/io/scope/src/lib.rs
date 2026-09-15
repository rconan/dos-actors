pub struct IOScope;
pub trait IOScopeTrait {
    type Signal;
}
#[cfg(signal)]
include!(concat!(env!("OUT_DIR"), "/scope.rs"));
