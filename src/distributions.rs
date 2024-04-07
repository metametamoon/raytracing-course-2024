use crate::bvh::{intersect_with_bvh_all_points, BvhTree};
use rand::Rng;
use rand_distr::{Distribution, Normal};

use crate::geometry::{
    intersect_ray_with_object3d_all_points, Fp, Object3D, Ray, Shape3D, Vec3f, EPS, FP_PI,
};

pub trait SampleDistribution<R: Rng> {
    fn sample_unit_vector(&self, point: &Vec3f, normal: &Vec3f, rng: &mut R) -> Vec3f;
    fn pdf(&self, point: &Vec3f, normal: &Vec3f, direction: &Vec3f) -> Fp;
}

pub struct SemisphereUniform;

pub struct CosineWeightedDistribution;

pub struct DirectLightSamplingDistribution {
    pub object3d: Object3D,
}

pub struct MultipleLightSamplingDistribution {
    pub bvh_tree: BvhTree,
}

pub struct MixDistribution<R: Rng> {
    pub distributions: Vec<Box<dyn SampleDistribution<R> + Sync>>,
}

impl<R: Rng> SampleDistribution<R> for SemisphereUniform {
    fn sample_unit_vector(&self, _: &Vec3f, normal: &Vec3f, rng: &mut R) -> Vec3f {
        let normal_distr = Normal::new(0.0, 1.0).unwrap();
        let result = Vec3f::new(
            normal_distr.sample(rng),
            normal_distr.sample(rng),
            normal_distr.sample(rng),
        )
        .normalize();
        if result.dot(normal) > 0.0 {
            result
        } else {
            -result
        }
    }

    fn pdf(&self, _point: &Vec3f, _normal: &Vec3f, _direction: &Vec3f) -> Fp {
        1.0 / (2.0 * FP_PI)
    }
}

impl<R: Rng> SampleDistribution<R> for CosineWeightedDistribution {
    fn sample_unit_vector(&self, _point: &Vec3f, normal: &Vec3f, rng: &mut R) -> Vec3f {
        let normal_distr = Normal::new(0.0, 1.0).unwrap();
        let uniform = Vec3f::new(
            normal_distr.sample(rng),
            normal_distr.sample(rng),
            normal_distr.sample(rng),
        )
        .normalize();
        let result = (uniform + normal).normalize();
        result
    }

    fn pdf(&self, _: &Vec3f, normal: &Vec3f, direction: &Vec3f) -> Fp {
        Fp::max(0.0, direction.normalize().dot(normal)) / FP_PI
    }
}

fn get_local_pdf(shape: &Shape3D, local_coords: Vec3f) -> Fp {
    match shape {
        Shape3D::Plane { .. } => 0.0,
        Shape3D::Ellipsoid { r } => {
            let n = local_coords.component_div(r);
            if (n.norm() - 1.0).abs() > EPS {
                log::debug!("Weird norm for unit sphere: {} of vector {:?}", n.norm(), n);
            }
            let root =
                ((n.x * r.y * r.z).powi(2) + (r.x * n.y * r.z).powi(2) + (r.x * r.y * n.z).powi(2))
                    .sqrt();
            1.0 / (4.0 * FP_PI * root)
        }
        Shape3D::Box { s } => {
            let area = (s.x * s.y + s.y * s.z + s.z * s.x) * 8.0;
            1.0 / area
        }
        Shape3D::Triangle { a, b, c } => {
            let area = (b - a).cross(&(c - a)).norm() * 0.5;
            1.0 / area
        }
    }
}

impl<R: Rng> SampleDistribution<R> for DirectLightSamplingDistribution {
    fn sample_unit_vector(&self, point: &Vec3f, _normal: &Vec3f, rng: &mut R) -> Vec3f {
        let local_coords_point = match self.object3d.shape {
            Shape3D::Plane { norm: _ } => panic!("Plane not supported!"),
            Shape3D::Ellipsoid { r } => {
                let normal_distr = Normal::new(0.0, 1.0).unwrap();
                let result = Vec3f::new(
                    normal_distr.sample(rng),
                    normal_distr.sample(rng),
                    normal_distr.sample(rng),
                )
                .normalize();
                result.component_mul(&r)
            }
            Shape3D::Box { s } => {
                let (wx, wy, wz) = (4.0 * s.y * s.z, 4.0 * s.x * s.z, 4.0 * s.x * s.y);
                let w = wx + wy + wz;
                let x = rng.gen_range(0.0..w);
                let rnd_sign = if fastrand::bool() { 1.0 } else { -1.0 };
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
            Shape3D::Triangle { a, b, c } => {
                let (u, v) = (rng.gen_range(0.0..1.0), rng.gen_range(0.0..1.0));
                let (u, v) = if u + v < 1.0 {
                    (u, v)
                } else {
                    (1.0 - u, 1.0 - v)
                };
                a + (b - a) * u + (c - a) * v
            }
        };
        let global_coords_point =
            self.object3d.rotation.transform_vector(&local_coords_point) + self.object3d.position;

        (global_coords_point - point).normalize()
    }

    fn pdf(&self, point: &Vec3f, _: &Vec3f, direction: &Vec3f) -> Fp {
        let ray = Ray {
            origin: *point,
            direction: *direction,
        };
        let (all_intersections, rotated_ray) =
            intersect_ray_with_object3d_all_points(&ray, &self.object3d);
        let pdf = all_intersections
            .iter()
            .map(|intersection| {
                let global_coords_point = ray.origin + intersection.offset * ray.direction;
                let local_coords = rotated_ray.origin + rotated_ray.direction * intersection.offset;
                let local_pdf = get_local_pdf(&self.object3d.shape, local_coords);
                let vector_on_sample = global_coords_point - point;
                let omega = vector_on_sample.normalize();
                local_pdf
                    * (vector_on_sample.norm_squared() / (intersection.normal.dot(&omega)).abs())
            })
            .sum();
        pdf
    }
}

impl<R: Rng> SampleDistribution<R> for MultipleLightSamplingDistribution {
    fn sample_unit_vector(&self, point: &Vec3f, normal: &Vec3f, rng: &mut R) -> Vec3f {
        let n = self.bvh_tree.primitives.len();
        let rand_idx = rng.gen_range(0..n);
        DirectLightSamplingDistribution {
            object3d: self.bvh_tree.primitives[rand_idx].object3d.clone(),
        }
        .sample_unit_vector(point, normal, rng)
    }

    fn pdf(&self, point: &Vec3f, _normal: &Vec3f, direction: &Vec3f) -> Fp {
        let ray = Ray {
            origin: *point,
            direction: *direction,
        };
        let bvh_intersections = intersect_with_bvh_all_points(&ray, &self.bvh_tree);
        let pdf = bvh_intersections
            .iter()
            .map(|bvh_intersection| {
                let rotated_ray = &bvh_intersection.rotated_ray;
                let object3d = &bvh_intersection.primitive.object3d;
                let mut sum = 0.0;
                for intersection in &bvh_intersection.intersections {
                    let global_coords_point = ray.origin + intersection.offset * ray.direction;
                    let local_coords =
                        rotated_ray.origin + rotated_ray.direction * intersection.offset;
                    let local_pdf = get_local_pdf(&object3d.shape, local_coords);
                    let vector_on_sample = global_coords_point - point;
                    let omega = vector_on_sample.normalize();
                    sum += local_pdf
                        * (vector_on_sample.norm_squared()
                            / (intersection.normal.dot(&omega)).abs())
                }
                sum
            })
            .sum::<Fp>();
        pdf / (self.bvh_tree.primitives.len() as Fp)
    }
}

impl<R: Rng> SampleDistribution<R> for MixDistribution<R> {
    fn sample_unit_vector(&self, point: &Vec3f, normal: &Vec3f, rng: &mut R) -> Vec3f {
        let n = self.distributions.len();
        let rand_idx = rng.gen_range(0..n);
        self.distributions[rand_idx].sample_unit_vector(point, normal, rng)
    }

    fn pdf(&self, point: &Vec3f, normal: &Vec3f, direction: &Vec3f) -> Fp {
        let ans = self
            .distributions
            .iter()
            .map(|distr| distr.pdf(point, normal, direction))
            .sum::<Fp>();
        ans / (self.distributions.len() as Fp)
    }
}
