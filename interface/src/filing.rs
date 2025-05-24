//! # Serialization interface
//!
//! Interface to serialize and to deserialize `dos-actors` objects.

use std::{
    env::{self, VarError},
    fmt::Debug,
    fs::File,
    io::{Read, Write},
    path::{Path, PathBuf},
};

use serde::{Deserialize, Serialize};

#[derive(Debug, thiserror::Error)]
pub enum FilingError {
    #[error("filing error")]
    IO(#[from] std::io::Error),
    #[error("can't create file {0:?}")]
    Create(#[source] std::io::Error, PathBuf),
    #[error("can't open file {0:?}")]
    Open(#[source] std::io::Error, PathBuf),
    #[cfg(feature = "filing")]
    #[error("decoder error")]
    Decoder(#[from] bincode::error::DecodeError),
    #[cfg(all(feature = "pickling", not(feature = "filing")))]
    #[error("decoder error")]
    PickleCodec(#[from] serde_pickle::error::Error),
    #[cfg(feature = "filing")]
    #[error("encoder error")]
    Encoder(#[from] bincode::error::EncodeError),
    #[error("builder error: {0}")]
    Builder(String),
    #[error("DATA_REPO not set")]
    DataRepo(#[from] VarError),
}

pub type Result<T> = std::result::Result<T, FilingError>;

/// Encoding and decoding
pub trait Codec
where
    Self: Sized + serde::ser::Serialize + for<'de> serde::de::Deserialize<'de>,
{
    /// Decodes object from [std::io::Read]
    #[inline]
    #[cfg(feature = "filing")]
    fn decode<R>(reader: &mut R) -> Result<Self>
    where
        R: Read,
    {
        Ok(bincode::serde::decode_from_std_read(
            reader,
            bincode::config::standard(),
        )?)
    }
    #[inline]
    #[cfg(all(feature = "pickling", not(feature = "filing")))]
    fn decode<R>(reader: &mut R) -> Result<Self>
    where
        R: Read,
    {
        Ok(serde_pickle::from_reader(reader, Default::default())?)
    }

    /// Encodes object to [std::io::Write]
    #[inline]
    #[cfg(feature = "filing")]
    fn encode<W>(&self, writer: &mut W) -> Result<()>
    where
        W: Write,
    {
        bincode::serde::encode_into_std_write(self, writer, bincode::config::standard())?;
        Ok(())
    }
    #[inline]
    #[cfg(all(feature = "pickling", not(feature = "filing")))]
    fn encode<W>(&self, writer: &mut W) -> Result<()>
    where
        W: Write,
    {
        serde_pickle::to_writer(writer, self, Default::default())?;
        Ok(())
    }
}

impl<T> Filing for T where
    T: Sized + Codec + serde::ser::Serialize + for<'de> serde::de::Deserialize<'de>
{
}

/// Encoding and decoding to/from [File]
pub trait Filing: Codec
where
    Self: Sized + serde::ser::Serialize + for<'de> serde::de::Deserialize<'de>,
{
    /// Decodes object from given path
    fn from_path<P>(path: P) -> Result<Self>
    where
        P: AsRef<Path> + Debug,
    {
        log::info!("decoding from {path:?}");
        let file =
            File::open(&path).map_err(|e| FilingError::Open(e, path.as_ref().to_path_buf()))?;
        let mut buffer = std::io::BufReader::new(file);
        Self::decode(&mut buffer)
    }

    /// Decodes object from given file
    /// The file is read from the directory specified by the `DATA_REPO` environment variable.
    fn from_data_repo(file_name: impl AsRef<Path>) -> Result<Self> {
        let data_repo = env::var("DATA_REPO")?;
        let path = Path::new(&data_repo).join(file_name);
        Self::from_path(path)
    }

    /// Encodes object from given path
    fn to_path<P>(&self, path: P) -> Result<()>
    where
        P: AsRef<Path> + Debug,
    {
        log::info!("encoding to {path:?}");
        let file =
            File::create(&path).map_err(|e| FilingError::Create(e, path.as_ref().to_path_buf()))?;
        let mut buffer = std::io::BufWriter::new(file);
        self.encode(&mut buffer)?;
        Ok(())
    }

    /// Encodes object from given file.
    /// The file is written to the directory specified by the `DATA_REPO` environment variable.
    fn to_data_repo(&self, file_name: impl AsRef<Path>) -> Result<()> {
        let data_repo = env::var("DATA_REPO")?;
        let path = Path::new(&data_repo).join(file_name);
        Self::to_path(self, path)
    }

    /// Decodes object from given path or creates a new object from the builder and encodes the object to the given path
    fn from_path_or_else<P, F, B>(path: P, builder: F) -> Result<Self>
    where
        P: AsRef<Path> + Debug,
        F: FnOnce() -> B,
        Self: TryFrom<B>,
        <Self as TryFrom<B>>::Error: std::fmt::Debug,
    {
        Self::from_path(&path).or_else(|e| {
            log::warn!("{e:?}");
            let this =
                Self::try_from(builder()).map_err(|e| FilingError::Builder(format!("{e:?}")))?;
            this.to_path(path)?;
            Ok(this)
        })
    }

    /// Decodes object from given file or creates a new object from the builder and encodes the object to the given file.
    /// The file is read from the directory specified by the `DATA_REPO` environment variable.
    fn from_data_repo_or_else<P, F, B>(file_name: P, builder: F) -> Result<Self>
    where
        P: AsRef<Path>,
        F: FnOnce() -> B,
        Self: TryFrom<B>,
        <Self as TryFrom<B>>::Error: std::fmt::Debug,
    {
        let data_repo = env::var("DATA_REPO")?;
        let path = Path::new(&data_repo).join(file_name);
        Self::from_path_or_else(path, builder)
    }

    /// Decodes object from given path or creates a new object from [Default] and encodes the object to the given path
    fn from_path_or_default<P, F, B>(path: P) -> Result<Self>
    where
        P: AsRef<Path> + Debug,
        B: Default,
        F: FnOnce() -> B,
        Self: TryFrom<B>,
        <Self as TryFrom<B>>::Error: std::fmt::Debug,
    {
        Self::from_path_or_else(path, Default::default)
    }

    /// Decodes object from given file or creates a new object from [Default] and encodes the object to the given file.
    /// The file is read from the directory specified by the `DATA_REPO` environment variable.
    fn from_data_repo_or_default<P, F, B>(file_name: P) -> Result<Self>
    where
        P: AsRef<Path>,
        B: Default,
        F: FnOnce() -> B,
        Self: TryFrom<B>,
        <Self as TryFrom<B>>::Error: std::fmt::Debug,
    {
        Self::from_data_repo_or_else(file_name, Default::default)
    }
    /// Loads an object and builder pair from a given path and returns the object
    /// only if the builder match the current one, or creates a new object from the
    /// current builder then encodes the new object and builder pair to the given path
    /// and finally returns the new object.
    fn from_path_or<P, B>(path: P, current_builder: B) -> Result<Self>
    where
        P: AsRef<Path> + Debug,
        Self: TryFrom<B> + serde::ser::Serialize + for<'de> serde::de::Deserialize<'de>,
        B: Clone + PartialEq + serde::ser::Serialize + for<'de> serde::de::Deserialize<'de>,
        <Self as TryFrom<B>>::Error: std::fmt::Debug,
    {
        match <ObjectAndBuilder<Self, B> as Filing>::from_path(&path) {
            Ok(ObjectAndBuilder { object, builder }) if builder == current_builder => Ok(object),
            _ => {
                let object = Self::try_from(current_builder.clone())
                    .map_err(|e| FilingError::Builder(format!("{e:?}")))?;
                let this = ObjectAndBuilder {
                    object,
                    builder: current_builder,
                };
                this.to_path(path)?;
                let ObjectAndBuilder { object, .. } = this;
                Ok(object)
            }
        }
    }
    /// Loads an object and builder pair from a given file and returns the object
    /// only if the builder match the current one, or creates a new object from the
    /// current builder then encodes the new object and builder pair to the given file
    /// and finally returns the new object.
    /// The file is read from the directory specified by the `DATA_REPO` environment variable.
    fn from_data_repo_or<P, B>(file_name: P, current_builder: B) -> Result<Self>
    where
        P: AsRef<Path>,
        Self: TryFrom<B> + serde::ser::Serialize + for<'de> serde::de::Deserialize<'de>,
        B: Clone + PartialEq + serde::ser::Serialize + for<'de> serde::de::Deserialize<'de>,
        <Self as TryFrom<B>>::Error: std::fmt::Debug,
    {
        let data_repo = env::var("DATA_REPO")?;
        let path = Path::new(&data_repo).join(file_name);
        Self::from_path_or(path, current_builder)
    }
}

/// Object and builder pair
#[derive(Serialize, Deserialize)]
struct ObjectAndBuilder<T, B>
where
    T: TryFrom<B>,
{
    object: T,
    builder: B,
}

impl<T, B> Codec for ObjectAndBuilder<T, B>
where
    T: TryFrom<B> + serde::ser::Serialize + for<'de> serde::de::Deserialize<'de>,
    B: serde::ser::Serialize + for<'de> serde::de::Deserialize<'de>,
{
}
