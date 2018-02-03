#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error {
    StartOutsideGrid,
    GoalOutsideGrid,
    StartSolid,
    NoPath,
}
