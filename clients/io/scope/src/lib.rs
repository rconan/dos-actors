pub struct IOScope;
pub trait IOScopeTrait {
    type Signal;
}
include!(concat!(env!("OUT_DIR"), "/scope.rs"));
