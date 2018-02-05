#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error {
    StartOutsideGrid,
    StartSolid,
    NoPath,
}
