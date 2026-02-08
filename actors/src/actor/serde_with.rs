use interface::TryUpdate;
use serde::{Deserialize, Deserializer, Serialize, Serializer};
use std::sync::Arc;
use tokio::{sync::Mutex, task};

pub fn serialize<S, C: TryUpdate + Serialize>(client: &Arc<Mutex<C>>, s: S) -> Result<S::Ok, S::Error>
where
    S: Serializer,
{
    task::block_in_place(move || client.blocking_lock().serialize(s))
}

pub fn deserialize<'de, D, C: TryUpdate + Deserialize<'de>>(
    deserializer: D,
) -> Result<Arc<Mutex<C>>, D::Error>
where
    D: Deserializer<'de>,
{
    Ok(Arc::new(Mutex::new(C::deserialize(deserializer)?)))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::prelude::Actor;

    #[derive(Debug, Serialize, Deserialize)]
    pub struct Client();
    impl interface::Update for Client {}

    #[test]
    fn serde() {
        let actor: Actor<_> = Client().into();
        dbg!(&actor);
        let value = serde_pickle::to_value(&actor).unwrap();
        let actor: Actor<Client, 1, 1> = serde_pickle::from_value(value).unwrap();
        dbg!(&actor);
    }
}
