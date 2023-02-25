use crate::types::{CCoord, WCoord};
use glam::{Mat3A, Vec3A};

#[derive(Clone, Copy, Debug)]
pub struct UV {
    u: Vec3A,
    v: Vec3A,
}

impl UV {
    #[allow(dead_code)]
    pub fn u(&self) -> &Vec3A {
        &self.u
    }

    #[allow(dead_code)]
    pub fn v(&self) -> &Vec3A {
        &self.v
    }

    /// Compute ∠(v, Ru)
    pub fn v_ru_angle(&self, rot: &Mat3A) -> f32 {
        self.v.angle_between(*rot * self.u)
    }
}

#[derive(Clone, Copy, Debug)]
pub struct Corres {
    projected: CCoord,
    world: WCoord,
}

impl Corres {
    pub fn new(projected: CCoord, world: WCoord) -> Self {
        Corres { projected, world }
    }

    pub fn compute_uv(&self, other: &Self) -> UV {
        let u = self.world - other.world;
        let v = self.projected.cross(other.projected);

        UV {
            u: u.as_vec3a(),
            v: v.as_vec3a(),
        }
    }

    #[allow(dead_code)]
    pub fn compute_translation(&self, rot: &Mat3A) -> Vec3A {
        self.projected.as_vec3a() - (*rot * self.world.as_vec3a())
    }
}

/// Represents two correspondences pair
#[derive(Clone, Copy, Debug)]
pub struct CorresPair<'a>(&'a Corres, &'a Corres);

impl<'a> CorresPair<'a> {
    /// Generates correspondence pairs from vector of correspondence
    pub fn make_pairs(corres: &'a [Corres]) -> Vec<CorresPair<'a>> {
        corres
            .chunks_exact(2)
            .map(|c| CorresPair(&c[0], &c[1]))
            .collect()
    }

    pub fn uv(&self) -> UV {
        self.0.compute_uv(self.1)
    }
}
