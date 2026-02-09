use std::{marker::PhantomData, sync::Arc};

use interface::TryUpdate;
use tokio::sync::Mutex;

use crate::{actor::Actor, ArcMutex};

pub struct Client<'a, T: ArcMutex> {
    client: Arc<Mutex<T>>,
    label: Option<String>,
    image: Option<String>,
    lifetime: PhantomData<&'a T>,
}

impl<T: TryUpdate> Client<'_, T> {
    pub fn set_label(&mut self, label: impl ToString) {
        self.label = Some(label.to_string());
    }
    pub fn set_image(&mut self, image: impl ToString) {
        self.image = Some(image.to_string());
    }
}

impl<T: TryUpdate> From<T> for Client<'_, T> {
    fn from(value: T) -> Self {
        Self {
            client: value.into_arcx(),
            label: None,
            image: None,
            lifetime: PhantomData,
        }
    }
}

impl<C: TryUpdate, const NI: usize, const NO: usize> From<&Client<'_, C>> for Actor<C, NI, NO> {
    fn from(client: &Client<C>) -> Self {
        let actor = Actor::new(client.client.clone());
        match (client.label.as_ref(), client.image.as_ref()) {
            (Some(label), Some(image)) => actor.name(label).image(image),
            (Some(label), None) => actor.name(label),
            (None, Some(image)) => actor.image(image),
            (None, None) => actor,
        }
    }
}

impl<'a, T: ArcMutex> Client<'a, T> {
    /// Locks inner client mutex
    pub async fn lock(&'a self) -> tokio::sync::MutexGuard<'a, T> {
        self.client.lock().await
    }
    /// Consumes the [Client], returning the inner client , if the [Arc](https://doc.rust-lang.org/std/sync/struct.Arc.html#method.into_inner) has exactly one strong reference.
    pub fn into_inner(self) -> Option<T> {
        Arc::into_inner(self.client).map(|inner| inner.into_inner())
    }
}

#[cfg(test)]
mod tests {
    use interface::Update;

    #[test]
    fn client() {
        use crate::{
            actor::Actor,
            client::{Client },
        };

        struct TestClient;

        impl Update for TestClient {}

        let test_client = TestClient;

        let client = Client::from(test_client);
        let actor: Actor<_> = Actor::from(&client);

        let other_client: Client<'_, TestClient> = client;
    }
}
