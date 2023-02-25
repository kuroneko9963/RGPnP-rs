use crate::corres::Corres;
use glam::Mat3A;

pub trait RBounds
where
    Self: Sized + Ord,
{
    fn upper(&self) -> u32;
    fn lower(&self) -> u32;

    fn subdivide(self) -> Vec<Self>;

    fn bounds(&mut self, corres: &[Corres], threshold: f32);
    fn rotation(&self) -> Mat3A;
}
