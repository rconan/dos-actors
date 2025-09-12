use std::sync::Arc;

use futures::{Stream, StreamExt, task::Poll};
use object_store::ObjectStore;

use super::{DomeSeeing, Result};
use crate::{DomeSeeingBuilder, DomeSeeingError, Opd};

impl DomeSeeing {
    /// Creates a [DomeSeeingBuilder] instance given the `path` to the CFD case
    pub fn builder(path: impl Into<object_store::path::Path>) -> DomeSeeingBuilder {
        DomeSeeingBuilder {
            cfd_case_path: path.into(),
            ..Default::default()
        }
    }
    pub async fn get_npyz_from_store(
        store: &Arc<dyn ObjectStore>,
        path: &object_store::path::Path,
    ) -> Result<Opd> {
        use std::io::Cursor;
        let mut data = Vec::new();
        let mut reader = store.get(&path).await?.into_stream();
        while let Some(chunk) = reader.next().await {
            let chunk = chunk?;
            data.extend_from_slice(&chunk);
        }

        let cursor = Cursor::new(data);

        let mut archive = npyz::npz::NpzArchive::new(cursor)
            .map_err(|e| DomeSeeingError::Load(e, path.to_string()))?;
        Self::get_npyz(path, &mut archive)
    }
    pub async fn get_npyz_from_store_owned(
        store: Arc<dyn ObjectStore>,
        path: object_store::path::Path,
    ) -> Result<Opd> {
        use std::io::Cursor;
        let mut data = Vec::new();
        let mut reader = store.get(&path).await?.into_stream();
        while let Some(chunk) = reader.next().await {
            let chunk = chunk?;
            data.extend_from_slice(&chunk);
        }

        let cursor = Cursor::new(data);

        let mut archive = npyz::npz::NpzArchive::new(cursor)
            .map_err(|e| DomeSeeingError::Load(e, path.to_string()))?;
        Self::get_npyz(path, &mut archive)
    }
    pub async fn get(&self, idx: usize) -> Result<Opd> {
        let path = &self
            .data
            .get(idx)
            .ok_or(DomeSeeingError::OutOfBounds(idx))?
            .file;

        Self::get_npyz_from_store(
            self.store.as_ref().ok_or(DomeSeeingError::MissingStore)?,
            path,
        )
        .await
    }
    pub async fn async_step(
        &mut self,
        data: Option<std::pin::Pin<Box<dyn std::future::Future<Output = Result<Opd>> + Send>>>,
    ) -> Vec<f64> {
        if let Some(data) = data {
            self.y2 = data.await.expect("failed to load dome seeing data file");
        }
        self.step()
    }
    pub async fn async_next(&mut self) -> Option<Vec<f64>> {
        let Some(opd) = futures::StreamExt::next(self).await else {
            return None;
        };
        Some(self.async_step(opd).await)
    }
}

impl Stream for DomeSeeing {
    type Item = Option<std::pin::Pin<Box<dyn std::future::Future<Output = Result<Opd>> + Send>>>;

    fn poll_next(
        mut self: std::pin::Pin<&mut Self>,
        _cx: &mut std::task::Context<'_>,
    ) -> Poll<Option<Self::Item>> {
        if self.i % self.upsampling == 0 {
            let y2 = self.y2.clone();
            let _ = std::mem::replace(&mut self.y1, y2);
            if let Some(idx) = self.counter.next() {
                let path = self
                    .data
                    .get(idx)
                    .unwrap()
                    .to_owned()
                    // .ok_or(DomeSeeingError::OutOfBounds(idx))?.ok()?
                    .file;
                let store = self
                    .store
                    .as_ref()
                    .unwrap()
                    // .ok_or(DomeSeeingError::MissingStore)?
                    .clone();
                Poll::Ready(
                    Some(Some(Box::pin(Self::get_npyz_from_store_owned(store, path)))), // .expect("failed to load dome seeing data file"),)
                )
            } else {
                Poll::Ready(None)
            }
        } else {
            Poll::Ready(Some(None))
        }
    }
}

#[cfg(test)]
mod tests {
    use std::env;

    use super::*;

    #[tokio::test]
    async fn load() -> std::result::Result<(), Box<dyn std::error::Error>> {
        use std::sync::Arc;
        let s3: Arc<dyn object_store::ObjectStore> = Arc::new(
            object_store::aws::AmazonS3Builder::from_env()
                .with_region("sa-east-1")
                .with_bucket_name("maua.cfd.2025")
                .build()?,
        );
        if let Ok(path) = env::var("CFD_PATH") {
            let dome_seeing = DomeSeeing::builder(path).store(s3.clone()).build().await?;
            assert!(
                dome_seeing.len() > 1,
                "expected dome seeing len > 1, found dome seeing len = {}",
                dome_seeing.len()
            );
        } else {
            println!("please set the env var CFD_PATH to the full path to a CFD case");
        }

        Ok(())
    }

    #[tokio::test]
    async fn time() -> std::result::Result<(), Box<dyn std::error::Error>> {
        use std::sync::Arc;
        let s3: Arc<dyn object_store::ObjectStore> = Arc::new(
            object_store::aws::AmazonS3Builder::from_env()
                .with_region("sa-east-1")
                .with_bucket_name("maua.cfd.2025")
                .build()?,
        );
        if let Ok(path) = env::var("CFD_PATH") {
            let dome_seeing = DomeSeeing::builder(path).store(s3.clone()).build().await?;
            assert!(dome_seeing[dome_seeing.len() - 1].time_stamp > dome_seeing[0].time_stamp);
        } else {
            println!("please set the env var CFD_PATH to the full path to a CFD case");
        }

        Ok(())
    }

    #[tokio::test]
    async fn next() -> std::result::Result<(), Box<dyn std::error::Error>> {
        use std::sync::Arc;
        let s3: Arc<dyn object_store::ObjectStore> = Arc::new(
            object_store::aws::AmazonS3Builder::from_env()
                .with_region("sa-east-1")
                .with_bucket_name("maua.cfd.2025")
                .build()?,
        );
        if let Ok(path) = env::var("CFD_PATH") {
            let mut dome_seeing = DomeSeeing::builder(path).store(s3.clone()).build().await?;
            for i in 0..5 {
                let mut opd = dome_seeing.async_next().await.unwrap();
                opd.sort_by(|a, b| a.partial_cmp(b).unwrap());
                println!(
                    "#{i:02}: [{:+6.0},{:+6.0}]nm",
                    opd[0] * 1e9,
                    1e9 * opd.last().unwrap()
                );
            }
        } else {
            println!("please set the env var CFD_PATH to the full path to a CFD case");
        }

        Ok(())
    }
}
