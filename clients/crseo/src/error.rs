#[non_exhaustive]
#[derive(thiserror::Error, Debug)]
pub enum CeoError {
    #[error("CEO building failed")]
    CEO(#[from] crseo::CrseoError),
}
pub type Result<T> = std::result::Result<T, CeoError>;
