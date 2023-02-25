use glam::Vec3A;

pub trait TBounds
where
    Self: Sized + Ord,
{
    fn upper(&self) -> u32;
    fn lower(&self) -> u32;

    fn subdivide(self) -> Vec<Self>;

    fn bounds(&mut self, trans: &[Self], threshold: f32);
    fn translation(&self) -> Vec3A;
}
