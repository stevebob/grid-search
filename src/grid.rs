pub trait SolidGrid {
    fn is_solid(&self, coord: (i32, i32)) -> bool;
}
