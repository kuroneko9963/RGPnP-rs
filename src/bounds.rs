mod rbound2;
mod rbounds;
mod tbounds;

use crate::corres::{Corres, CorresPair};
use glam::{Mat3A, Vec3, Vec3A};
use std::cmp::Ordering;
use std::f32::consts::{FRAC_PI_2, PI};
use std::ops::RangeInclusive;

pub use rbound2::RBound;
pub use rbounds::RBounds;
pub use tbounds::TBounds;

const F32_2PI: f32 = 2.0_f32 * PI;

pub struct RBAngleAxis {
    upper: u32,
    lower: u32,
    center: Vec3A,
    edge: f32,
}

#[derive(Debug)]
pub struct AngleRange {
    pub min: f32,
    pub max: f32,
}

impl AngleRange {
    pub fn center(&self) -> f32 {
        (self.min + self.max) / 2.0
    }

    pub fn length(&self) -> f32 {
        (self.max - self.min).abs()
    }
}

impl<T> From<RangeInclusive<T>> for AngleRange
where
    T: Into<f32>,
{
    fn from(value: RangeInclusive<T>) -> Self {
        let (min, max) = value.into_inner();
        let (min, max) = (min.into(), max.into());

        AngleRange { min, max }
    }
}

pub struct RBPolar {
    upper: u32,
    lower: u32,
    theta: AngleRange,
    phi: AngleRange,
}

impl RBAngleAxis {
    pub fn new(center: Vec3A, edge: f32) -> Self {
        RBAngleAxis {
            upper: 0,
            lower: 0,
            center,
            edge,
        }
    }
}

impl Default for RBAngleAxis {
    fn default() -> Self {
        RBAngleAxis {
            upper: 0,
            lower: 0,
            center: Vec3A::ZERO,
            edge: F32_2PI,
        }
    }
}

impl PartialEq for RBAngleAxis {
    fn eq(&self, other: &Self) -> bool {
        if self.edge.is_nan() || other.edge.is_nan() {
            panic!("edge is nan.");
        }

        self.upper == other.upper && self.edge == other.edge && self.lower == other.lower
    }
}

impl Eq for RBAngleAxis {}

impl PartialOrd for RBAngleAxis {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for RBAngleAxis {
    fn cmp(&self, other: &Self) -> Ordering {
        if self.edge.is_nan() || other.edge.is_nan() {
            panic!("edge is nan.");
        }

        if self == other {
            Ordering::Equal
        } else if self.upper < other.upper
            || (self.upper == other.upper && self.edge < other.edge)
            || (self.upper == other.upper && self.edge == other.edge && self.lower < other.lower)
        {
            Ordering::Less
        } else {
            Ordering::Greater
        }
    }
}

impl RBounds for RBAngleAxis {
    fn upper(&self) -> u32 {
        self.upper
    }

    fn lower(&self) -> u32 {
        self.lower
    }

    fn subdivide(self) -> Vec<Self> {
        let [cx, cy, cz] = self.center.to_array();
        let half = self.edge / 2.0;
        let quat = self.edge / 4.0;

        // 1e-8
        // if quat < f32::EPSILON {
        //     return vec![];
        // }

        let center_points = vec![
            (cx + quat, cy + quat, cz + quat).into(),
            (cx + quat, cy + quat, cz - quat).into(),
            (cx + quat, cy - quat, cz + quat).into(),
            (cx + quat, cy - quat, cz - quat).into(),
            (cx - quat, cy + quat, cz + quat).into(),
            (cx - quat, cy + quat, cz - quat).into(),
            (cx - quat, cy - quat, cz + quat).into(),
            (cx - quat, cy - quat, cz - quat).into(),
        ];

        center_points
            .into_iter()
            .map(|cp| RBAngleAxis::new(cp, half))
            .collect()
    }

    fn bounds(&mut self, corres: &[Corres], threshold: f32) {
        for c_pair in CorresPair::make_pairs(corres) {
            let angle = c_pair.uv().v_ru_angle(&self.rotation());
            let error = (angle - FRAC_PI_2).abs();
            let alpha = f32::sqrt(3.0) * (self.edge / 2.0);

            if error < threshold + alpha {
                self.upper += 1;
            }
            if error < threshold {
                self.lower += 1;
            }
        }
    }

    fn rotation(&self) -> glam::Mat3A {
        let angle = self.center.length();
        let axis = self.center.try_normalize().unwrap_or(Vec3A::X).into();

        Mat3A::from_axis_angle(axis, angle)
    }
}

impl RBPolar {
    pub fn new(theta: impl Into<AngleRange>, phi: impl Into<AngleRange>) -> Self {
        let theta = theta.into();
        let phi = phi.into();

        RBPolar {
            upper: 0,
            lower: 0,
            theta,
            phi,
        }
    }
}

impl Default for RBPolar {
    fn default() -> Self {
        RBPolar {
            upper: 0,
            lower: 0,
            theta: (-PI..=PI).into(),
            phi: (-FRAC_PI_2..=FRAC_PI_2).into(),
        }
    }
}

impl PartialEq for RBPolar {
    fn eq(&self, other: &Self) -> bool {
        if self.theta.length().is_nan() || other.theta.length().is_nan() {
            panic!("theta is nan.");
        }
        if self.phi.length().is_nan() || other.phi.length().is_nan() {
            panic!("phi is nan.");
        }

        self.upper == other.upper
            && self.theta.length() == other.theta.length()
            && self.phi.length() == other.phi.length()
            && self.lower == other.lower
    }
}

impl Eq for RBPolar {}

impl PartialOrd for RBPolar {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for RBPolar {
    fn cmp(&self, other: &Self) -> Ordering {
        if self.theta.length().is_nan() || other.theta.length().is_nan() {
            panic!("theta is nan.");
        }
        if self.phi.length().is_nan() || other.phi.length().is_nan() {
            panic!("phi is nan.");
        }

        if self == other {
            Ordering::Equal
        } else if self.upper < other.upper
            || (self.upper == other.upper && self.theta.length() < other.theta.length())
            || (self.upper == other.upper
                && self.theta.length() == other.theta.length()
                && self.phi.length() < other.phi.length())
            || (self.upper == other.upper
                && self.theta.length() == other.theta.length()
                && self.phi.length() == other.phi.length()
                && self.lower < other.lower)
        {
            Ordering::Less
        } else {
            Ordering::Greater
        }
    }
}

impl RBounds for RBPolar {
    fn upper(&self) -> u32 {
        self.upper
    }

    fn lower(&self) -> u32 {
        self.lower
    }

    fn subdivide(self) -> Vec<Self> {
        // 5e-2, 5e-2
        // if self.theta.length() < EPSILON && self.phi.length() < EPSILON {
        //     return vec![];
        // }
        // if self.theta.length() < f32::EPSILON {
        //     return vec![];
        // }

        let (t_min, t_max, t_center) = (self.theta.min, self.theta.max, self.theta.center());
        let (p_min, p_max, p_center) = (self.phi.min, self.phi.max, self.phi.center());
        let next_angle = vec![
            (t_min..=t_center, p_min..=p_center),
            (t_min..=t_center, p_center..=p_max),
            (t_center..=t_max, p_min..=p_center),
            (t_center..=t_max, p_center..=p_max),
        ];

        next_angle
            .into_iter()
            .map(|(next_theta, next_phi)| RBPolar::new(next_theta, next_phi))
            .collect()
    }

    fn bounds(&mut self, corres: &[Corres], threshold: f32) {
        for c_pair in CorresPair::make_pairs(corres) {
            let angle = c_pair.uv().v_ru_angle(&self.rotation());
            let error = (angle - FRAC_PI_2).abs();
            let alpha = f32::acos(
                f32::cos(self.theta.length() / 2.0) + f32::cos(self.phi.length() / 2.0) - 1.0,
            );

            if error < threshold + alpha {
                self.upper += 1;
            }
            if error < threshold {
                self.lower += 1;
            }
        }
    }

    fn rotation(&self) -> Mat3A {
        let rz = Mat3A::from_axis_angle(Vec3::Z, self.theta.center());
        let ry = Mat3A::from_axis_angle(Vec3::Y, self.phi.center());

        ry * rz
    }
}
