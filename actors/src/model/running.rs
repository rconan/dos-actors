use super::{Completed, Model, ModelError, Result, Running};
use crate::{
    framework::model::TaskError::FromActor,
    ActorError::{Disconnected, DropRecv, DropSend},
};
use chrono::{DateTime, Local, SecondsFormat};
use std::{
    future::{Future, IntoFuture},
    marker::PhantomData,
    pin::Pin,
    time::Instant,
};

impl Model<Running> {
    /// Waits for the task of each actor to finish
    pub async fn wait(mut self) -> Result<Model<Completed>> {
        let mut task_set = self.task_set.take().unwrap();
        // for task_handle in task_set.into_iter() {
        while let Some(task_handle) = task_set.join_next().await {
            // task_handle.await?.map_err(|e| Box::new(e))?;
            match task_handle? {
                Ok(_) => {
                    log::debug!(
                        "{} succesfully completed",
                        self.name.as_ref().unwrap_or(&String::from("Model"))
                    );
                    Ok(())
                }
                Err(FromActor(Disconnected(msg))) => {
                    log::debug!("{} has been disconnected", msg);
                    Ok(())
                }
                Err(FromActor(DropRecv { msg, .. })) => {
                    log::debug!("{} has been dropped", msg);
                    Ok(())
                }
                Err(FromActor(DropSend { msg, .. })) => {
                    log::debug!("{} has been dropped", msg);
                    Ok(())
                }
                Err(e) => Err(e),
            }
            .map_err(Box::new)?;
        }
        let elapsed_time = Instant::now().duration_since(self.start);
        let now: DateTime<Local> = Local::now();
        self.verbose.then(|| {
            eprintln!(
                "[{}<{}>] COMPLETED in {}",
                self.name
                    .as_ref()
                    .unwrap_or(&String::from("Model"))
                    .to_uppercase(),
                now.to_rfc3339_opts(SecondsFormat::Secs, true),
                humantime::format_duration(elapsed_time)
            )
        });
        Ok(Model::<Completed> {
            name: self.name,
            actors: None,
            task_set: None,
            state: PhantomData,
            start: Instant::now(),
            verbose: self.verbose,
            elapsed_time: elapsed_time.as_secs_f64(),
        })
    }
}

pub type ModelCompleted = Pin<
    Box<dyn Future<Output = std::result::Result<Model<Completed>, ModelError>> + Send + 'static>,
>;
impl IntoFuture for Model<Running> {
    type IntoFuture = ModelCompleted;
    type Output = <ModelCompleted as Future>::Output;
    fn into_future(self) -> Self::IntoFuture {
        Box::pin(self.wait())
    }
}
