use super::{Model, Ready, Running};
use chrono::{DateTime, Local, SecondsFormat};
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
        let task_handles: Vec<_> = self
            .actors
            .into_iter()
            .flatten()
            .map(|mut actor| tokio::spawn(async move { actor.task().await }))
            .collect();
        Model::<Running> {
            name: self.name,
            actors: None,
            task_handles: Some(task_handles),
            state: PhantomData,
            start: Instant::now(),
            verbose: self.verbose,
         }
    }
}
