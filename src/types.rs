use glam::{IVec2, Vec3A};
use std::ops::Sub;

/// Represents world coordinate
#[derive(Clone, Copy, Debug)]
pub struct WCoord(Vec3A);

/// Represents camera coordinate
#[derive(Clone, Copy, Debug)]
pub struct CCoord(Vec3A);

/// Represents image plane coordinate
#[derive(Clone, Copy, Debug)]
pub struct ICoord(IVec2);

impl WCoord {
    pub fn as_vec3a(self) -> Vec3A {
        self.0
    }
}

impl Sub for WCoord {
    type Output = Self;
    fn sub(self, rhs: Self) -> Self::Output {
        WCoord(self.0 - rhs.0)
    }
}

impl<T> From<T> for WCoord
where
    T: Into<Vec3A>,
{
    fn from(value: T) -> Self {
        WCoord(value.into())
    }
}

impl CCoord {
    pub fn as_vec3a(self) -> Vec3A {
        self.0
    }
    /// Computes cross product
    pub fn cross(self, rhs: Self) -> Self {
        CCoord(self.0.cross(rhs.0))
    }
}

impl<T> From<T> for CCoord
where
    T: Into<Vec3A>,
{
    fn from(value: T) -> Self {
        CCoord(value.into())
    }
}

impl ICoord {
    pub fn as_ivec2(self) -> IVec2 {
        self.0
    }
}

impl<T> From<T> for ICoord
where
    T: Into<IVec2>,
{
    fn from(value: T) -> Self {
        ICoord(value.into())
    }
}
