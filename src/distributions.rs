use rand::Rng;

use rand_distr::{Distribution, Normal};

use crate::geometry::{intersect_ray_with_object3d_all_points, Object3D, Ray, Shape3D, Vec3f, EPS};

pub trait SampleDistribution {
    fn sample(&self, point: &Vec3f, normal: &Vec3f) -> Vec3f;
    fn pdf(&self, point: &Vec3f, normal: &Vec3f, direction: &Vec3f) -> f64;
}

pub struct SemisphereUniform;

pub struct CosineWeightedDistribution;

pub struct LightSamplingDistribution {
    pub object3d: Object3D,
}

pub struct MixDistribution {
    pub distributions: Vec<Box<dyn SampleDistribution>>,
}

impl SampleDistribution for SemisphereUniform {
    fn sample(&self, _: &Vec3f, normal: &Vec3f) -> Vec3f {
        let mut rng = rand::thread_rng();
        let normal_distr = Normal::new(0.0, 1.0).unwrap();
        let result = Vec3f::new(
            normal_distr.sample(&mut rng),
            normal_distr.sample(&mut rng),
            normal_distr.sample(&mut rng),
        )
        .normalize();
        if result.dot(normal) > 0.0 {
            result
        } else {
            -result
        }
    }

    fn pdf(&self, _point: &Vec3f, _normal: &Vec3f, _direction: &Vec3f) -> f64 {
        1.0 / (2.0 * std::f64::consts::PI)
    }
}

impl SampleDistribution for CosineWeightedDistribution {
    fn sample(&self, point: &Vec3f, normal: &Vec3f) -> Vec3f {
        let mut rng = rand::thread_rng();
        let normal_distr = Normal::new(0.0, 1.0).unwrap();
        let uniform = Vec3f::new(
            normal_distr.sample(&mut rng),
            normal_distr.sample(&mut rng),
            normal_distr.sample(&mut rng),
        )
        .normalize();
        let result = (uniform + normal).normalize();
        assert!(self.pdf(point, normal, &result) > 0.0);
        result
    }

    fn pdf(&self, _: &Vec3f, normal: &Vec3f, direction: &Vec3f) -> f64 {
        f64::max(0.0, direction.normalize().dot(normal)) / std::f64::consts::PI
    }
}

fn get_local_pdf(shape: &Shape3D, local_coords: Vec3f) -> f64 {
    match shape {
        Shape3D::None => 0.0,
        Shape3D::Plane { .. } => 0.0,
        Shape3D::Ellipsoid { r } => {
            let n = local_coords.component_div(r);
            if (n.norm() - 1.0).abs() > EPS {
                log::debug!("Weird norm for unit sphere: {} of vector {:?}", n.norm(), n);
            }
            let root =
                ((n.x * r.y * r.z).powi(2) + (r.x * n.y * r.z).powi(2) + (r.x * r.y * n.z).powi(2))
                    .sqrt();
            1.0 / (4.0 * std::f64::consts::PI * root)
        }
        Shape3D::Box { s } => {
            let area = (s.x * s.y + s.y * s.z + s.z * s.x) * 8.0;
            1.0 / area
        }
    }
}

impl SampleDistribution for LightSamplingDistribution {
    fn sample(&self, point: &Vec3f, _normal: &Vec3f) -> Vec3f {
        let mut rng = rand::thread_rng();
        let local_coords_point = match self.object3d.shape {
            Shape3D::None => panic!("Shape None!"),
            Shape3D::Plane { norm: _ } => panic!("Plane not supported!"),
            Shape3D::Ellipsoid { r } => {
                let normal_distr = Normal::new(0.0, 1.0).unwrap();
                let result = Vec3f::new(
                    normal_distr.sample(&mut rng),
                    normal_distr.sample(&mut rng),
                    normal_distr.sample(&mut rng),
                )
                .normalize();
                result.component_mul(&r)
            }
            Shape3D::Box { s } => {
                let (wx, wy, wz) = (4.0 * s.y * s.z, 4.0 * s.x * s.z, 4.0 * s.x * s.y);
                let w = wx + wy + wz;
                let x = rng.gen_range(0.0..w);
                let rnd_sign = if rng.gen_bool(0.5) { 1.0 } else { -1.0 };
                if x < wx {
                    Vec3f::new(
                        s.x * rnd_sign,
                        rng.gen_range(-s.y..s.y),
                        rng.gen_range(-s.z..s.z),
                    )
                } else if x < wx + wy {
                    Vec3f::new(
                        rng.gen_range(-s.x..s.x),
                        s.y * rnd_sign,
                        rng.gen_range(-s.z..s.z),
                    )
                } else {
                    Vec3f::new(
                        rng.gen_range(-s.x..s.x),
                        rng.gen_range(-s.y..s.y),
                        s.z * rnd_sign,
                    )
                }
            }
        };
        let global_coords_point =
            self.object3d.rotation.transform_vector(&local_coords_point) + self.object3d.position;

        global_coords_point - point
    }

    fn pdf(&self, point: &Vec3f, _: &Vec3f, direction: &Vec3f) -> f64 {
        let ray = Ray {
            origin: *point,
            direction: *direction,
        };
        let all_intersections = intersect_ray_with_object3d_all_points(&ray, &self.object3d);
        let pdf = all_intersections
            .iter()
            .map(|intersection| {
                let global_coords_point = ray.origin + intersection.offset * ray.direction;
                let local_coords = self
                    .object3d
                    .rotation
                    .conjugate()
                    .transform_vector(&(global_coords_point - self.object3d.position));
                let local_pdf = get_local_pdf(&self.object3d.shape, local_coords);
                let vector_on_sample = global_coords_point - point;
                let omega = vector_on_sample.normalize();
                // println!("Intersection normal: {:?}", intersection.normal);
                local_pdf
                    * (vector_on_sample.norm_squared() / (intersection.normal.dot(&omega)).abs())
            })
            .sum();
        pdf
    }
}

impl SampleDistribution for MixDistribution {
    fn sample(&self, point: &Vec3f, normal: &Vec3f) -> Vec3f {
        let mut rng = rand::thread_rng();
        let n = self.distributions.len();
        let rand_idx = rng.gen_range(0..n);
        self.distributions[rand_idx].sample(point, normal)
    }

    fn pdf(&self, point: &Vec3f, normal: &Vec3f, direction: &Vec3f) -> f64 {
        let ans = self
            .distributions
            .iter()
            .map(|distr| distr.pdf(point, normal, direction))
            .sum::<f64>();
        ans / (self.distributions.len() as f64)
    }
}
