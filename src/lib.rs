#![feature(binary_heap_retain)]

mod bnb;
mod bounds;
// mod bounds2;
mod bounds3;
mod corres;

pub mod types;

// use bounds::{RBAngleAxis, RBPolar};
// use bounds2::{RBAngleAxis, RBPolarWithAngle};
// use bounds2::{RBAngleAxis, RBPolar, RBPolarWithAngle};
use bounds3::{RBAngleAxis, RBPolar};
use corres::Corres;
use glam::{Mat3A, Vec3A};
use std::f32::consts::{FRAC_PI_2, PI};
use types::{CCoord, ICoord, WCoord};

use crate::bounds::{RBound, RBounds};

const F32_2PI: f32 = 2.0 * PI;

#[derive(Clone, Copy, Debug)]
pub enum RotationBound {
    AngleAxis,
    PolarCoordinate,
}

#[derive(Clone, Copy, Debug)]
pub struct CameraK {
    pub fx: f32,
    pub fy: f32,
    pub cx: f32,
    pub cy: f32,
}

impl CameraK {
    pub fn as_mat3a(self) -> Mat3A {
        Mat3A::from_cols(
            Vec3A::X * self.fx, // [self.fx,     0.0, 0.0]
            Vec3A::Y * self.fy, // [    0.0, self.fy, 0.0]
            [self.cx, self.cy, 1.0].into(),
        )
    }

    pub fn to_camera_coord(&self, image: &ICoord) -> CCoord {
        (self.as_mat3a().inverse() * image.as_ivec2().as_vec2().extend(1.0)).into()
    }
}

impl From<Mat3A> for CameraK {
    fn from(value: Mat3A) -> Self {
        CameraK {
            fx: value.x_axis.x,
            fy: value.y_axis.y,
            cx: value.z_axis.x,
            cy: value.z_axis.y,
        }
    }
}

pub struct Solver {
    corres: Vec<Corres>,
    r_threshold: f32,
    #[allow(dead_code)]
    t_threshold: f32,
    rot_bound: RotationBound,
}

impl Solver {
    pub fn new(r_threshold: f32, t_threshold: f32) -> Self {
        Solver {
            corres: vec![],
            r_threshold,
            t_threshold,
            rot_bound: RotationBound::AngleAxis,
        }
    }

    pub fn rot_bound(&mut self) -> &mut RotationBound {
        &mut self.rot_bound
    }

    pub fn reset_correspondence(&mut self) {
        self.corres.clear();
    }

    pub fn reserve_correspondence(&mut self, n: usize) {
        self.corres.reserve(n);
    }

    pub fn add_correspondence(&mut self, projected: ICoord, world: WCoord, k: &CameraK) {
        let camera_coord = k.to_camera_coord(&projected);

        self.corres.push(Corres::new(camera_coord, world));
    }

    pub fn pose(&self) -> (Mat3A, Vec3A) {
        #[cfg(debug_assertions)]
        match self.rot_bound {
            RotationBound::AngleAxis => println!("Mode: angle-axis based"),
            RotationBound::PolarCoordinate => println!("Mode: polar based"),
        }

        let rot = match self.rot_bound {
            RotationBound::AngleAxis => {
                // bnb::bnb_rot(
                //     RBAngleAxis::new(Vec3A::ZERO, F32_2PI),
                //     &self.corres,
                //     self.r_threshold,
                // )
                // bnb::bnb_rot(
                //     RBAngleAxis::new(Vec3A::ZERO, F32_2PI, &self.corres),
                //     &self.corres,
                //     self.r_threshold,
                // )
                bnb::bnb_rot3(
                    vec![RBAngleAxis::new(Vec3A::ZERO, F32_2PI, &self.corres)],
                    self.r_threshold,
                )
            }
            RotationBound::PolarCoordinate => {
                // bnb::bnb_rot(RBPolar::default(), &self.corres, self.r_threshold)
                // bnb::bnb_rot2(
                //     vec![RBPolarWithAngle::new(
                //         0.0..=F32_2PI,
                //         -PI..=PI,
                //         -FRAC_PI_2..=FRAC_PI_2,
                //         &self.corres,
                //     )],
                //     self.r_threshold,
                // )
                // .first()
                // .map(RBound::rotation)
                // .unwrap_or(Mat3A::IDENTITY)
                // const N: i32 = 16;
                // let initial_rotations = (0..N)
                //     .map(|i| {
                //         (
                //             (i as f32) * F32_2PI / (N as f32),
                //             ((i + 1) as f32) * F32_2PI / (N as f32),
                //         )
                //     })
                //     .map(|(a_min, a_max)| {
                //         RBPolar::new(
                //             a_min..=a_max,
                //             -PI..=PI,
                //             -FRAC_PI_2..=FRAC_PI_2,
                //             &self.corres,
                //         )
                //     })
                //     .map(|mut bound| {
                //         bound.compute_bound(self.r_threshold);
                //         bound
                //     })
                //     .collect();
                // let bounds = bnb::bnb_rot2(initial_rotations, self.r_threshold)
                //     .into_iter()
                //     .map(RBPolarWithAngle::from)
                //     .collect();
                // bnb::bnb_rot2(bounds, self.r_threshold)
                //     .first()
                //     .map(RBound::rotation)
                //     .unwrap_or(Mat3A::IDENTITY)
                bnb::bnb_rot3(
                    vec![RBPolar::new(
                        -PI..=PI,
                        -FRAC_PI_2..=FRAC_PI_2,
                        -PI..=PI,
                        &self.corres,
                    )],
                    self.r_threshold,
                )
            }
        };

        #[cfg(debug_assertions)]
        println!("to estimate of translation has not been implemented yet...");

        (rot, Vec3A::ZERO)
    }
}
