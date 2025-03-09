use super::{Data, Read, UniqueIdentifier, Update};
use std::fmt::Display;

/// Simple data logging
///
/// Accumulates all the inputs in a single [Vec]
#[derive(Debug)]
pub struct Logging<T> {
    data: Vec<T>,
    n_sample: usize,
    n_entry: usize,
}

impl<T> std::ops::Deref for Logging<T> {
    type Target = Vec<T>;
    fn deref(&self) -> &Self::Target {
        &self.data
    }
}

impl<T> Default for Logging<T> {
    fn default() -> Self {
        Self {
            n_entry: 1,
            data: Vec::new(),
            n_sample: 0,
        }
    }
}
impl<T> Logging<T> {
    /// Creates a new data logger for a given number of entries
    pub fn new(n_entry: usize) -> Self {
        Self {
            n_entry,
            ..Default::default()
        }
    }
    #[deprecated(note = "please use `new` instead")]
    /// Sets the # of entries to be logged (default: 1)
    pub fn n_entry(self, n_entry: usize) -> Self {
        Self { n_entry, ..self }
    }
    /// Pre-allocates the size of the vector holding the data
    pub fn capacity(self, capacity: usize) -> Self {
        Self {
            data: Vec::with_capacity(capacity),
            ..self
        }
    }
    /// Returns the # of time samples
    pub fn len(&self) -> usize {
        self.n_sample / self.n_entry
    }
    /// Returns the sum of the entry sizes
    pub fn n_data(&self) -> usize {
        self.data.len() / self.len()
    }
    /// Checks if the logger is empty
    pub fn is_empty(&self) -> bool {
        self.n_sample == 0
    }
    /// Returns data chunks the size of the entries
    pub fn chunks(&self) -> impl Iterator<Item = &[T]> {
        self.data.chunks(self.n_data())
    }
    #[cfg(feature = "matio-rs")]
    /// Saves the data to a Matlab mat file
    pub fn to_mat_file<'a, S>(&'a self, file_name: S) -> Result<&'a Self, matio_rs::MatioError>
    where
        matio_rs::Mat<'a>: matio_rs::MayBeFrom<matio_rs::MatArray<'a, T>>,
        S: AsRef<std::path::Path>,
    {
        matio_rs::MatFile::save(file_name)?.array(
            "data",
            self.data.as_slice(),
            vec![self.n_data() as u64, self.len() as u64],
        )?;
        Ok(self)
    }
}

impl<T> Display for Logging<T> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        writeln!(
            f,
            "Logging: ({}x{})={}",
            self.n_data(),
            self.len(),
            self.data.len()
        )
    }
}

impl<T> Update for Logging<T> where T: Send + Sync {}
impl<T, U> Read<U> for Logging<T>
where
    T: Clone + Send + Sync,
    U: UniqueIdentifier<DataType = Vec<T>>,
{
    fn read(&mut self, data: Data<U>) {
        // log::debug!("receive {} input: {:}", type_name::<U>(), data.len(),);
        self.data.extend((&**data).to_vec());
        self.n_sample += 1;
    }
}
