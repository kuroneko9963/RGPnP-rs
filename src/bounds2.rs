use crate::bounds::RBound;
use crate::bounds::RBounds;
use crate::corres::{Corres, CorresPair};
use glam::{Mat3A, Vec3, Vec3A};
use std::cmp::Ordering;
use std::f32::consts::{FRAC_PI_2, PI};
use std::ops::RangeInclusive;

pub struct RBAngleAxis<'a> {
    upper: u32,
    lower: u32,
    center: Vec3A,
    edge: f32,
    corres_pairs: Vec<CorresPair<'a>>,
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
        let edge_equal = self.edge.total_cmp(&other.edge).is_eq();

        self.upper == other.upper && edge_equal && self.lower == other.lower
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

impl<'a> RBounds for RBAngleAxis<'a> {
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

    fn bounds(&mut self, _corres: &[Corres], threshold: f32) {
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

    fn rotation(&self) -> Mat3A {
        let Some(axis) = self.center.try_normalize() else {
            return Mat3A::IDENTITY;
        };
        let angle = self.center.length();

        Mat3A::from_axis_angle(axis.into(), angle)
    }
}

#[derive(Clone, Copy, Debug)]
pub struct Range {
    min: f32,
    max: f32,
}

impl Range {
    pub fn center(&self) -> f32 {
        (self.min + self.max) / 2.0
    }

    pub fn length(&self) -> f32 {
        (self.max - self.min).abs()
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

pub struct RBPolar<'a> {
    upper: u32,
    lower: u32,
    angle: Range,
    theta: Range,
    phi: Range,
    corres_pairs: Vec<CorresPair<'a>>,
}

impl<'a> RBPolar<'a> {
    pub fn new(
        angle: impl Into<Range>,
        theta: impl Into<Range>,
        phi: impl Into<Range>,
        corres: &'a [Corres],
    ) -> Self {
        let angle = angle.into();
        let theta = theta.into();
        let phi = phi.into();
        let corres_pairs = CorresPair::make_pairs(corres);

        RBPolar {
            upper: 0,
            lower: 0,
            angle,
            theta,
            phi,
            corres_pairs,
        }
    }
}

impl<'a> PartialEq for RBPolar<'a> {
    fn eq(&self, other: &Self) -> bool {
        self.upper == other.upper
            && self.angle == other.angle
            && self.theta == other.theta
            && self.phi == other.phi
            && self.lower == other.lower
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
        if self == other {
            Ordering::Equal
        } else if self.upper < other.upper
            || (self.upper == other.upper && self.theta < other.theta)
            || (self.upper == other.upper && self.theta == other.theta && self.phi < other.phi)
            || (self.upper == other.upper
                && self.theta == other.theta
                && self.phi == other.phi
                && self.lower < other.lower)
        {
            Ordering::Less
        } else {
            Ordering::Greater
        }
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
            .map(|(next_theta, next_phi)| RBPolar {
                upper: 0,
                lower: 0,
                angle: self.angle,
                theta: next_theta.into(),
                phi: next_phi.into(),
                corres_pairs: self.corres_pairs.clone(),
            })
            .collect()
    }

    fn rotation(&self) -> Mat3A {
        let theta = self.theta.center();
        let phi = self.phi.center();
        let axis = (
            theta.sin() * phi.cos(),
            theta.sin() * phi.sin(),
            theta.cos(),
        );

        Mat3A::from_axis_angle(axis.into(), self.angle.center())
    }

    fn compute_bound(&mut self, threshold: f32) {
        for c_pair in &self.corres_pairs {
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
}

pub struct RBPolarWithAngle<'a> {
    upper: u32,
    lower: u32,
    angle: Range,
    theta: Range,
    phi: Range,
    corres_pairs: Vec<CorresPair<'a>>,
}

impl<'a> RBPolarWithAngle<'a> {
    pub fn new<A, T, P>(angle: A, theta: T, phi: P, corres: &'a [Corres]) -> Self
    where
        A: Into<Range>,
        T: Into<Range>,
        P: Into<Range>,
    {
        let angle = angle.into();
        let theta = theta.into();
        let phi = phi.into();
        let corres_pairs = CorresPair::make_pairs(corres);

        RBPolarWithAngle {
            upper: 0,
            lower: 0,
            angle,
            theta,
            phi,
            corres_pairs,
        }
    }
}

impl<'a> PartialEq for RBPolarWithAngle<'a> {
    fn eq(&self, other: &Self) -> bool {
        self.upper == other.upper
            && self.angle == other.angle
            && self.theta == other.theta
            && self.phi == other.phi
            && self.lower == other.lower
    }
}
impl<'a> Eq for RBPolarWithAngle<'a> {}

impl<'a> PartialOrd for RBPolarWithAngle<'a> {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}
impl<'a> Ord for RBPolarWithAngle<'a> {
    fn cmp(&self, other: &Self) -> Ordering {
        if self == other {
            Ordering::Equal
        } else if self.upper < other.upper
            || (self.upper == other.upper && self.angle < other.angle)
            || (self.upper == other.upper && self.angle == other.angle && self.theta < other.theta)
            || (self.upper == other.upper
                && self.angle == other.angle
                && self.theta == other.theta
                && self.phi < other.phi)
            || (self.upper == other.upper
                && self.angle == other.angle
                && self.theta == other.theta
                && self.phi == other.phi
                && self.lower < other.lower)
        {
            Ordering::Less
        } else {
            Ordering::Greater
        }
    }
}

impl<'a> RBounds for RBPolarWithAngle<'a> {
    fn upper(&self) -> u32 {
        self.upper
    }

    fn lower(&self) -> u32 {
        self.lower
    }

    fn subdivide(self) -> Vec<Self> {
        let (a_min, a_max, ac) = (self.angle.min, self.angle.max, self.angle.center());
        let (t_min, t_max, tc) = (self.theta.min, self.theta.max, self.theta.center());
        let (p_min, p_max, pc) = (self.phi.min, self.phi.max, self.phi.center());

        let next_rot = vec![
            (a_min..=ac, t_min..=tc, p_min..=pc),
            (ac..=a_max, t_min..=tc, p_min..=pc),
            (a_min..=ac, t_min..=tc, pc..=p_max),
            (ac..=a_max, t_min..=tc, pc..=p_max),
            (a_min..=ac, tc..=t_max, p_min..=pc),
            (ac..=a_max, tc..=t_max, p_min..=pc),
            (a_min..=ac, tc..=t_max, pc..=p_max),
            (ac..=a_max, tc..=t_max, pc..=p_max),
        ];

        next_rot
            .into_iter()
            .map(|(angle, theta, phi)| RBPolarWithAngle {
                upper: 0,
                lower: 0,
                angle: angle.into(),
                theta: theta.into(),
                phi: phi.into(),
                corres_pairs: self.corres_pairs.clone(),
            })
            .collect()
    }

    fn bounds(&mut self, _corres: &[Corres], threshold: f32) {
        for c_pair in &self.corres_pairs {
            let angle = c_pair.uv().v_ru_angle(&RBounds::rotation(self));
            let error = (angle - FRAC_PI_2).abs();
            let alpha =
                ((self.theta.length() / 2.0).cos() + (self.phi.length() / 2.0).cos() - 1.0).acos();

            if error < threshold + alpha {
                self.upper += 1;
            }
            if error < threshold {
                self.lower += 1;
            }
        }
    }

    fn rotation(&self) -> Mat3A {
        let (ts, tc) = self.theta.center().sin_cos();
        let (ps, pc) = self.phi.center().sin_cos();
        let axis = Vec3::new(ts * pc, ts * ps, tc);

        Mat3A::from_axis_angle(axis, self.angle.center())
    }
}

impl<'a> RBound for RBPolarWithAngle<'a> {
    fn upper(&self) -> u32 {
        self.upper
    }

    fn lower(&self) -> u32 {
        self.lower
    }

    fn subdivide(self) -> Vec<Self> {
        let (a_min, a_max, ac) = (self.angle.min, self.angle.max, self.angle.center());
        let (t_min, t_max, tc) = (self.theta.min, self.theta.max, self.theta.center());
        let (p_min, p_max, pc) = (self.phi.min, self.phi.max, self.phi.center());

        let next_rot = vec![
            (a_min..=ac, t_min..=tc, p_min..=pc),
            (ac..=a_max, t_min..=tc, p_min..=pc),
            (a_min..=ac, t_min..=tc, pc..=p_max),
            (ac..=a_max, t_min..=tc, pc..=p_max),
            (a_min..=ac, tc..=t_max, p_min..=pc),
            (ac..=a_max, tc..=t_max, p_min..=pc),
            (a_min..=ac, tc..=t_max, pc..=p_max),
            (ac..=a_max, tc..=t_max, pc..=p_max),
        ];

        next_rot
            .into_iter()
            .map(|(angle, theta, phi)| RBPolarWithAngle {
                upper: 0,
                lower: 0,
                angle: angle.into(),
                theta: theta.into(),
                phi: phi.into(),
                corres_pairs: self.corres_pairs.clone(),
            })
            .collect()
    }

    fn compute_bound(&mut self, threshold: f32) {
        for c_pair in &self.corres_pairs {
            let uv = c_pair.uv();
            let rot = RBound::rotation(self);
            let angle = uv.v_ru_angle(&rot);
            let error = (angle - FRAC_PI_2).abs();
            // let alpha =
            //     ((self.theta.length() / 2.0).cos() + (self.phi.length() / 2.0).cos() - 1.0).acos();
            let alpha = {
                let theta = self.theta.center();
                let phi = self.phi.center();
                let axis = Vec3::new(theta.sin() * phi.cos(), theta.sin() * phi.sin(), phi.cos());
                let d = Mat3A::from_axis_angle(axis, self.angle.length() / 2.0);
                let u = uv.u();

                (rot * *u).angle_between(d * rot * *u)
            };

            if error < threshold + alpha {
                self.upper += 1;
            }
            if error < threshold {
                self.lower += 1;
            }
        }
    }

    fn rotation(&self) -> Mat3A {
        let (ts, tc) = self.theta.center().sin_cos();
        let (ps, pc) = self.phi.center().sin_cos();
        let axis = Vec3::new(ts * pc, ts * ps, tc);

        Mat3A::from_axis_angle(axis, self.angle.center())
    }
}

impl<'a> From<RBPolar<'a>> for RBPolarWithAngle<'a> {
    fn from(value: RBPolar<'a>) -> Self {
        RBPolarWithAngle {
            upper: 0,
            lower: 0,
            angle: value.angle,
            theta: value.theta,
            phi: value.phi,
            corres_pairs: value.corres_pairs,
        }
    }
}
