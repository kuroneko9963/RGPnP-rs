use glam::{Mat3A, Vec3A};
use rand::Rng;
use rand_distr::Normal;
use rgpnp_rs::types::{ICoord, WCoord};
use rgpnp_rs::{CameraK, RotationBound, Solver};
use std::time::Instant;

#[allow(dead_code)]
enum CameraMode {
    LeftHand,
    RightHand,
}

struct GaussianNoise {
    distribution: Normal<f32>,
}

impl GaussianNoise {
    fn new(mean: f32, std_dev: f32) -> Self {
        GaussianNoise {
            distribution: Normal::new(mean, std_dev).unwrap(),
        }
    }

    fn gen(&self, rng: &mut impl Rng) -> (f32, f32, f32) {
        (
            rng.sample(self.distribution),
            rng.sample(self.distribution),
            rng.sample(self.distribution),
        )
    }
}

struct Camera {
    rotation: Mat3A,
    translation: Vec3A,
    direction: Vec3A,
}

impl Camera {
    fn look(
        at: impl Into<Vec3A>,
        center: impl Into<Vec3A>,
        up: impl Into<Vec3A>,
        mode: CameraMode,
    ) -> Camera {
        let at: Vec3A = at.into();
        let center: Vec3A = center.into();
        let up: Vec3A = up.into();

        let dir = center - at;
        let dir = match mode {
            CameraMode::LeftHand => dir.normalize(),
            CameraMode::RightHand => (-dir).normalize(),
        };
        let side = up.cross(dir).normalize();
        let up = dir.cross(side).normalize();

        Camera {
            rotation: Mat3A::from_cols(side, up, dir),
            translation: center,
            direction: dir,
        }
    }

    fn project(
        &self,
        p: impl Into<WCoord>,
        k: &CameraK,
        noise: GaussianNoise,
        rng: &mut impl Rng,
        image_size: (i32, i32),
    ) -> Option<ICoord> {
        let p: WCoord = p.into();

        // projects the point `p` if `p` is located in front of this camera.
        if self.direction.dot(p.as_vec3a()) <= 0.0 {
            return None;
        }

        let camera = self.rotation * p.as_vec3a() + self.translation;
        let mut image = k.as_mat3a() * camera + Vec3A::from(noise.gen(rng));
        image /= image.z;

        // returns `None` if the projected point is outside the image plane.
        let image = image.round().truncate().as_ivec2();
        let (image_w, image_h) = image_size;
        if !(0..image_w).contains(&image.x) || !(0..image_h).contains(&image.y) {
            return None;
        }

        Some(image.into())
    }
}

fn main() {
    const N: usize = 100;
    const FX: f32 = 718.856;
    const FY: f32 = 718.856;
    const CX: f32 = 607.1928;
    const CY: f32 = 185.2157;
    const IMAGE_W: i32 = 1241;
    const IMAGE_H: i32 = 376;
    const WORLD_X: (f32, f32) = (-2.0, 2.0);
    const WORLD_Y: (f32, f32) = (-2.0, 2.0);
    const WORLD_Z: (f32, f32) = (4.0, 8.0);
    // const SMALL_X: (f32, f32) = (1.0, 2.0);
    // const SMALL_Y: (f32, f32) = (1.0, 2.0);
    // const SMALL_Z: (f32, f32) = (4.0, 5.0);
    const R_THRESHOLD: f32 = 0.2;
    const T_THRESHOLD: f32 = 0.2;

    let mut a_solver = {
        let mut s = Solver::new(R_THRESHOLD, T_THRESHOLD);
        *s.rot_bound() = RotationBound::AngleAxis;
        s.reserve_correspondence(N);
        s
    };
    let mut p_solver = {
        let mut s = Solver::new(R_THRESHOLD, T_THRESHOLD);
        *s.rot_bound() = RotationBound::PolarCoordinate;
        s.reserve_correspondence(N);
        s
    };

    let camera = Camera::look(
        (0.0, 0.0, 6.0),
        (2.0_f32.sqrt() / 2.0, 2.0_f32.sqrt() / 2.0, 0.0), // Vec3A::ZERO,
        Vec3A::Y,
        CameraMode::RightHand,
    );
    let k = CameraK {
        fx: FX,
        fy: FY,
        cx: CX,
        cy: CY,
    };

    let mut rng = rand::thread_rng();

    let standard_deviations = [0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0, 4.5, 5.0];
    for std_dev in standard_deviations {
        //
        // Experiment 1. Ordinary 3D area
        //

        // generate random world points and corresponded image points
        let mut correspondences = Vec::with_capacity(N);
        let mut n = 0;
        while n < N {
            let wx = rng.gen_range(WORLD_X.0..=WORLD_X.1);
            let wy = rng.gen_range(WORLD_Y.0..=WORLD_Y.1);
            let wz = rng.gen_range(WORLD_Z.0..=WORLD_Z.1);
            let world = (wx, wy, wz);

            if let Some(image) = camera.project(
                world,
                &k,
                GaussianNoise::new(0.0, std_dev),
                &mut rng,
                (IMAGE_W, IMAGE_H),
            ) {
                correspondences.push((WCoord::from(world), image));
                n += 1;
            }
        }

        // solve
        for (world, image) in correspondences {
            a_solver.add_correspondence(image, world, &k);
            p_solver.add_correspondence(image, world, &k);
        }

        let a_timer = Instant::now();
        let (a_rot, _) = a_solver.pose();
        let a_elapsed = a_timer.elapsed().as_millis();

        let p_timer = Instant::now();
        let (p_rot, _) = p_solver.pose();
        let p_elapsed = p_timer.elapsed().as_millis();

        let a_err = f32::acos((a_rot * Vec3A::X).dot(camera.rotation * Vec3A::X));
        let p_err = f32::acos((p_rot * Vec3A::X).dot(camera.rotation * Vec3A::X));

        // println!("ground truth: {}", camera.rotation);
        // println!("AngleAxis   : {}", a_rot);
        // println!("Polar       : {}", p_rot);

        println!(
            "Noise Level: {:>4.1} - AngleAxis [ elapsed: {:>8}ms, error: {:>8.6} ], Polar [ elapsed: {:>8}ms, error: {:>8.6} ]",
            std_dev,
            a_elapsed,
            a_err,
            p_elapsed,
            p_err
        );

        //
        // Experiment 2. Quasi-singular area
        //
    }
}
