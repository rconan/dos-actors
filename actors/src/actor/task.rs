use async_trait::async_trait;
use cudarc::driver::CudaContext;
use interface::TryUpdate;
use std::{any::type_name, sync::Arc};
use tokio::sync::Mutex;

use crate::framework::model::{Task, TaskError};

use super::{Actor, PlainActor};

type Result<T> = std::result::Result<T, TaskError>;

#[async_trait]
impl<C, const NI: usize, const NO: usize> Task for Actor<C, NI, NO>
where
    C: 'static + TryUpdate,
{
    /// Run the actor loop
    async fn task(mut self: Box<Self>) -> Result<()> {
        /*         match self.bootstrap().await {
            Err(e) => crate::print_info(
                format!("{} bootstrapping failed", Who::highlight(self)),
                Some(&e),
            ),
            Ok(_) => {
                crate::print_info(
                    format!("{} loop started", Who::highlight(self)),
                    None::<&dyn std::error::Error>,
                );
                if let Err(e) = self.async_run().await {
                    println!(
                        "{}{:?}",
                        format!("{} loop ended", Who::highlight(self)),
                        Some(&e)
                    );
                }
            }
        } */
        self.async_run().await
    }
    async fn blocking_task(mut self: Box<Self>) -> std::result::Result<(), TaskError> {
        self.blocking_async_run().await
    }

    /// Starts the actor infinite loop
    async fn async_run(&mut self) -> Result<()> {
        log::debug!("ACTOR LOOP ({NI}/{NO}): {}", type_name::<C>());
        let bootstrap = self.bootstrap().await?;
        let cuda = self.cuda.clone();
        match (self.inputs.as_ref(), self.outputs.as_ref()) {
            (Some(_), Some(_)) => {
                if NO >= NI {
                    // Decimation
                    if !bootstrap {
                        // bootstrap is applied when decimation is used
                        // in conjunction with averaging
                        // When averaging there is a delay of `NO` samples
                        // to account for the time to iterate and a default
                        // values is used for the 1st output
                        // For decimation of the input signal there is no delay
                        // and the 1st sample goes through unimpeded
                        self.collect().await?;
                        {
                            let mut client = self.client.lock().await;
                            cuda.as_ref().map(|cuda| cuda.bind_to_thread());
                            client.boxed_try_update()?;
                        }
                        self.distribute().await?;
                    }
                    loop {
                        for _ in 0..NO / NI {
                            self.collect().await?;
                            {
                                let mut client = self.client.lock().await;
                                cuda.as_ref().map(|cuda| cuda.bind_to_thread());
                                client.boxed_try_update()?;
                            }
                        }
                        self.distribute().await?;
                    }
                } else {
                    // Upsampling
                    loop {
                        self.collect().await?;
                        {
                            let mut client = self.client.lock().await;
                            cuda.as_ref().map(|cuda| cuda.bind_to_thread());
                            client.boxed_try_update()?;
                        }
                        for _ in 0..NI / NO {
                            self.distribute().await?;
                        }
                    }
                }
            }
            (None, Some(_)) => {
                // Initiator
                tokio::task::yield_now().await; // at least cooperates with other tasks
                loop {
                    {
                        let mut client = self.client.lock().await;
                        cuda.as_ref().map(|cuda| cuda.bind_to_thread());
                        client.boxed_try_update()?;
                    }
                    self.distribute().await?;
                }
            }
            (Some(_), None) => loop {
                // Terminator
                self.collect().await?;
                {
                    let mut client = self.client.lock().await;
                    cuda.as_ref().map(|cuda| cuda.bind_to_thread());
                    client.boxed_try_update()?;
                }
            },
            (None, None) => Ok(()),
        }
    }

    async fn blocking_async_run(&mut self) -> Result<()> {
        log::debug!("ACTOR LOOP ({NI}/{NO}): {}", type_name::<C>());
        let try_update = |client: std::sync::Arc<Mutex<C>>, cuda: Option<Arc<CudaContext>>| {
            tokio::task::spawn_blocking(move || {
                let mut c = client.blocking_lock();
                cuda.as_ref().map(|cuda| cuda.bind_to_thread());
                let _ = c.boxed_try_update()?;
                Ok::<(), TaskError>(())
            })
        };
        let bootstrap = self.bootstrap().await?;
        match (self.inputs.as_ref(), self.outputs.as_ref()) {
            (Some(_), Some(_)) => {
                if NO >= NI {
                    // Decimation
                    if !bootstrap {
                        // bootstrap is applied when decimation is used
                        // in conjunction with averaging
                        // When averaging there is a delay of `NO` samples
                        // to account for the time to iterate and a default
                        // values is used for the 1st output
                        // For decimation of the input signal there is no delay
                        // and the 1st sample goes through unimpeded
                        self.collect().await?;
                        try_update(self.client.clone(), self.cuda.clone()).await??;
                        self.distribute().await?;
                    }
                    loop {
                        for _ in 0..NO / NI {
                            self.collect().await?;
                            try_update(self.client.clone(), self.cuda.clone()).await??;
                        }
                        self.distribute().await?;
                    }
                } else {
                    // Upsampling
                    loop {
                        self.collect().await?;
                        try_update(self.client.clone(), self.cuda.clone()).await??;
                        for _ in 0..NI / NO {
                            self.distribute().await?;
                        }
                    }
                }
            }
            (None, Some(_)) => {
                // Initiator
                tokio::task::yield_now().await; // at least cooperates with other tasks
                loop {
                    try_update(self.client.clone(), self.cuda.clone()).await??;
                    self.distribute().await?;
                }
            }
            (Some(_), None) => loop {
                // Terminator
                self.collect().await?;
                try_update(self.client.clone(), self.cuda.clone()).await??;
            },
            (None, None) => Ok(()),
        }
    }

    fn as_plain(&self) -> PlainActor {
        self.into()
    }
    fn name(&self) -> &'static str {
        type_name::<C>()
    }
    fn blocking(&self) -> bool {
        self.blocking
    }
}
