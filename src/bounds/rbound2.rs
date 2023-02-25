use glam::Mat3A;

pub trait RBound
where
    Self: Sized + Ord,
{
    fn upper(&self) -> u32;
    fn lower(&self) -> u32;

    fn subdivide(self) -> Vec<Self>;

    fn rotation(&self) -> Mat3A;
    fn compute_bound(&mut self, threshold: f32);
}
