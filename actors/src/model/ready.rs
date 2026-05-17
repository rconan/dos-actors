use super::{Model, Ready, Running};
use chrono::{DateTime, Local, SecondsFormat};
use tokio::task::JoinSet;
use std::{marker::PhantomData, time::Instant};

impl Model<Ready> {
    /// Spawns each actor task
    pub fn run(self) -> Model<Running> {
        let now: DateTime<Local> = Local::now();
        self.verbose.then(|| {
            eprintln!(
                "[{}<{}>] LAUNCHED",
                self.name
                    .as_ref()
                    .unwrap_or(&String::from("Model"))
                    .to_uppercase(),
                now.to_rfc3339_opts(SecondsFormat::Secs, true),
            )
        });
        // let mut task_handles = vec![];
        /*         let mut actors = self.actors.take().unwrap();
        while let Some(mut actor) = actors.pop() {
            task_handles.push(tokio::spawn(async move {
                actor.task().await;
            }));
        } */
        let mut set = JoinSet::new();
        let _task_handles: Vec<_> = self
            .actors
            .into_iter()
            .flatten()
            .map(|actor| {
                #[cfg(tokio_unstable)]
                {
                    set.build_task()
                        .name(actor.name())
                        .spawn(async move { actor.task().await })
                        .unwrap()
                }
                #[cfg(not(tokio_unstable))]
                set.spawn(async move { actor.task().await })
            })
            .collect();
        Model::<Running> {
            name: self.name,
            actors: None,
            task_set: Some(set),
            state: PhantomData,
            start: Instant::now(),
            verbose: self.verbose,
            elapsed_time: Default::default(),
        }
    }
}
