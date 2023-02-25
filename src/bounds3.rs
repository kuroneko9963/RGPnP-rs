use crate::bounds::RBound;
use crate::corres::{Corres, CorresPair};
use glam::{Mat3A, Vec3, Vec3A};
use std::cmp::Ordering;
use std::f32::consts::{FRAC_PI_2, PI};
use std::ops::RangeInclusive;

const F32_2PI: f32 = 2.0 * PI;

#[derive(Clone, Copy, Debug)]
pub struct Range {
    min: f32,
    max: f32,
}

pub struct RBAngleAxis<'a> {
    upper: u32,
    lower: u32,
    center: Vec3A,
    edge: f32,
    corres_pairs: Vec<CorresPair<'a>>,
}

pub struct RBPolar<'a> {
    upper: u32,
    lower: u32,
    theta: Range,
    phi: Range,
    angle: Range,
    corres_pairs: Vec<CorresPair<'a>>,
}

impl Range {
    pub fn center(&self) -> f32 {
        (self.min + self.max) / 2.0
    }

    pub fn length(&self) -> f32 {
        (self.max - self.min).abs()
    }

    pub fn divide(&self) -> (Self, Self) {
        (
            Range {
                min: self.min,
                max: self.center(),
            },
            Range {
                min: self.center(),
                max: self.max,
            },
        )
    }

    pub fn cos_nearby(&self, a: f32) -> f32 {
        let acos = a.acos();
        let range = self.min..=self.max;
        if range.contains(&acos) {
            acos
        } else if range.contains(&(-acos)) {
            -acos
        } else {
            let min_cos = self.min.cos();
            let max_cos = self.max.cos();
            if (min_cos - a).abs() < (max_cos - a).abs() {
                min_cos
            } else {
                max_cos
            }
        }
    }
}

impl PartialEq for Range {
    fn eq(&self, other: &Self) -> bool {
        self.length().total_cmp(&other.length()).is_eq()
    }
}
impl Eq for Range {}

impl PartialOrd for Range {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}
impl Ord for Range {
    fn cmp(&self, other: &Self) -> Ordering {
        self.length().total_cmp(&other.length())
    }
}

impl From<RangeInclusive<f32>> for Range {
    fn from(value: RangeInclusive<f32>) -> Self {
        let (min, max) = value.into_inner();

        Range { min, max }
    }
}

impl<'a> RBAngleAxis<'a> {
    pub fn new(center: Vec3A, edge: f32, corres: &'a [Corres]) -> Self {
        let corres_pairs = CorresPair::make_pairs(corres);

        RBAngleAxis {
            upper: 0,
            lower: 0,
            center,
            edge,
            corres_pairs,
        }
    }
}

impl<'a> PartialEq for RBAngleAxis<'a> {
    fn eq(&self, other: &Self) -> bool {
        self.upper == other.upper
    }
}
impl<'a> Eq for RBAngleAxis<'a> {}

impl<'a> PartialOrd for RBAngleAxis<'a> {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}
impl<'a> Ord for RBAngleAxis<'a> {
    fn cmp(&self, other: &Self) -> Ordering {
        match (self.upper.cmp(&other.upper), self.lower.cmp(&other.lower)) {
            (Ordering::Equal, lower_ordering) => lower_ordering,
            (upper_ordering, _) => upper_ordering,
        }
    }
}

impl<'a> RBound for RBAngleAxis<'a> {
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

        let center_points = vec![
            (cx + quat, cy + quat, cz + quat),
            (cx + quat, cy + quat, cz - quat),
            (cx + quat, cy - quat, cz + quat),
            (cx + quat, cy - quat, cz - quat),
            (cx - quat, cy + quat, cz + quat),
            (cx - quat, cy + quat, cz - quat),
            (cx - quat, cy - quat, cz + quat),
            (cx - quat, cy - quat, cz - quat),
        ];
        center_points
            .into_iter()
            .map(|cp| RBAngleAxis {
                upper: 0,
                lower: 0,
                center: cp.into(),
                edge: half,
                corres_pairs: self.corres_pairs.clone(),
            })
            .collect()
    }

    fn rotation(&self) -> Mat3A {
        let Some(axis) = self.center.try_normalize() else {
            return Mat3A::IDENTITY;
        };
        let axis = axis.into();
        let angle = self.center.length();

        Mat3A::from_axis_angle(axis, angle)
    }

    fn compute_bound(&mut self, threshold: f32) {
        for c_pair in &self.corres_pairs {
            let angle = c_pair.uv().v_ru_angle(&self.rotation());
            let error = (angle - FRAC_PI_2).abs();
            let alpha = 3.0_f32.sqrt() * (self.edge / 2.0);

            if error < threshold + alpha {
                self.upper += 1;
            }
            if error < threshold {
                self.lower += 1;
            }
        }
    }
}

impl<'a> RBPolar<'a> {
    pub fn new(
        theta: impl Into<Range>,
        phi: impl Into<Range>,
        angle: impl Into<Range>,
        corres: &'a [Corres],
    ) -> Self {
        let theta = theta.into();
        let phi = phi.into();
        let angle = angle.into();
        let corres_pairs = CorresPair::make_pairs(corres);

        RBPolar {
            upper: 0,
            lower: 0,
            theta,
            phi,
            angle,
            corres_pairs,
        }
    }
}

impl<'a> PartialEq for RBPolar<'a> {
    fn eq(&self, other: &Self) -> bool {
        self.upper == other.upper
    }
}
impl<'a> Eq for RBPolar<'a> {}

impl<'a> PartialOrd for RBPolar<'a> {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}
impl<'a> Ord for RBPolar<'a> {
    fn cmp(&self, other: &Self) -> Ordering {
        // match (self.upper.cmp(&other.upper), self.lower.cmp(&other.lower)) {
        //     (Ordering::Equal, lower_ordering) => lower_ordering,
        //     (upper_ordering, _) => upper_ordering,
        // }
        self.upper.cmp(&other.upper)
    }
}

impl<'a> RBound for RBPolar<'a> {
    fn upper(&self) -> u32 {
        self.upper
    }

    fn lower(&self) -> u32 {
        self.lower
    }

    fn subdivide(self) -> Vec<Self> {
        let (t1, t2) = self.theta.divide();
        let (p1, p2) = self.phi.divide();
        let (a1, a2) = self.angle.divide();

        let next = vec![
            (t1, p1, a1),
            (t1, p1, a2),
            (t1, p2, a1),
            (t1, p2, a2),
            (t2, p1, a1),
            (t2, p1, a2),
            (t2, p2, a1),
            (t2, p2, a2),
        ];
        next.into_iter()
            .map(|(theta, phi, angle)| RBPolar {
                upper: 0,
                lower: 0,
                theta,
                phi,
                angle,
                corres_pairs: self.corres_pairs.clone(),
            })
            .collect()
    }

    fn rotation(&self) -> Mat3A {
        let t = self.theta.center();
        let p = self.phi.center();
        let axis = Vec3::new(t.sin() * p.cos(), t.sin() * p.sin(), t.cos());
        let angle = self.angle.center();

        Mat3A::from_axis_angle(axis, angle)
    }

    fn compute_bound(&mut self, threshold: f32) {
        for c_pair in &self.corres_pairs {
            let uv = c_pair.uv();
            let u = uv.u();
            let v = uv.v();
            // let angle = uv.v_ru_angle(&self.rotation());
            let r0u = self.rotation() * *u;
            let angle = v.angle_between(r0u);
            let error = (angle - FRAC_PI_2).abs();
            let alpha = {
                // let r1 = self.angle.center();
                // let r2 = self.angle.max;
                // let t1 = self.theta.center();
                // let t2 = {
                //     if t1.cos() > 0.0 {
                //         self.theta.cos_nearby(-1.0)
                //     } else {
                //         self.theta.cos_nearby(1.0)
                //     }
                // };
                // let p1 = self.phi.center();
                // let p2 = {
                //     let d_phi = Range {
                //         min: -self.phi.length() / 2.0,
                //         max: self.phi.length() / 2.0,
                //     };
                //     if t1.sin() * t2.sin() > 0.0 {
                //         d_phi.cos_nearby(-1.0)
                //     } else {
                //         d_phi.cos_nearby(1.0)
                //     }
                // };

                // (r1.powi(2) + r2.powi(2)
                //     - 2.0 * r1 * r2 * ((p1 - p2).cos() * t1.sin() * t2.sin() + t1.cos() * t2.cos()))
                // .sqrt()
                // let r = Mat3A::from_axis_angle(
                //     Vec3::new(t2.sin() * p2.cos(), t2.sin() * p2.sin(), t2.cos()),
                //     (r1.powi(2) + r2.powi(2)
                //         - 2.0
                //             * r1
                //             * r2
                //             * ((p1 - p2).cos() * t1.sin() * t2.sin() + t1.cos() * t2.cos()))
                //     .sqrt(),
                // );
                // let ru = r * *u;
                // r0u.angle_between(ru).abs()
                // {
                //     let r = self.angle.center();
                //     let a = self.angle.max;
                //     let t = self.theta.length() / 2.0;
                //     let p = self.phi.length() / 2.0;

                //     (r.powi(2) + a.powi(2) - 2.0 * r * a * (t.cos() + p.cos() - 1.0)).sqrt()
                // }
                self.angle.length() * self.theta.length() * self.phi.length() / 8.0
            };

            if error < threshold + alpha {
                self.upper += 1;
            }
            if error < threshold {
                self.lower += 1;
            }

            // println!(
            //     "{:?}, {:?}, {:?}, upper={}, lower={}, alpha={}",
            //     self.theta, self.phi, self.angle, self.upper, self.lower, alpha
            // );
        }
    }
}
