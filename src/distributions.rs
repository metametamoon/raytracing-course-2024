use rand::Rng;

use rand_distr::{Distribution, Normal};

use crate::geometry::{
    intersect_ray_with_primitive, Intersection, Primitive, Ray, Shape3D, Vec3f, EPS,
};

pub trait SampleDistribution {
    fn sample(&self, point: &Vec3f, normal: &Vec3f) -> Vec3f;
    fn pdf(&self, point: &Vec3f, normal: &Vec3f, direction: &Vec3f) -> f64;
}

pub struct SemisphereUniform;

pub struct CosineWeightedDistribution;

pub struct DirectionOnObjectDistribution {
    pub primitive: Primitive,
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
        if result.dot(&normal) > 0.0 {
            result
        } else {
            -result
        }
    }

    fn pdf(&self, point: &Vec3f, normal: &Vec3f, direction: &Vec3f) -> f64 {
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
        f64::max(0.0, direction.dot(&normal)) / std::f64::consts::PI
    }
}

impl SampleDistribution for DirectionOnObjectDistribution {
    fn sample(&self, point: &Vec3f, normal: &Vec3f) -> Vec3f {
        let mut rng = rand::thread_rng();
        let local_coords_point = match self.primitive.shape {
            Shape3D::None => panic!("Shape None!"),
            Shape3D::Plane { norm } => {
                panic!("Plane not supported!")
            }
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
        let global_coords_point = self
            .primitive
            .rotation
            .transform_vector(&local_coords_point)
            + self.primitive.position;
        let direction = global_coords_point - point;
        if let None = intersect_ray_with_primitive(
            &Ray {
                origin: point.clone(),
                direction,
            },
            &self.primitive,
            f64::INFINITY,
        ) {
            log::debug!("ray end point:{:?}", point.clone() + direction);
            let x = intersect_ray_with_primitive(
                &Ray {
                    origin: point.clone(),
                    direction,
                },
                &self.primitive,
                f64::INFINITY,
            );
            log::debug!("direction on vec turned into 0");
        }
        if !(self.pdf(&point, &normal, &direction) > 0.0) {
            let _ = self.pdf(&point, &normal, &direction);
        }
        direction
    }

    fn pdf(&self, point: &Vec3f, _: &Vec3f, direction: &Vec3f) -> f64 {
        let first_ray = Ray {
            origin: *point,
            direction: *direction,
        };
        if let Some(intersection) =
            intersect_ray_with_primitive(&first_ray, &self.primitive, f64::INFINITY)
        {
            match self.primitive.shape {
                Shape3D::None => panic!("pdf on None!"),
                Shape3D::Plane { norm: _ } => panic!("pdf on Plane!"),
                Shape3D::Ellipsoid { r } => {
                    let get_pdf = |r: &Vec3f, ray: &Ray, intersection: &Intersection| {
                        let global_coords_point = ray.origin + intersection.offset * ray.direction;
                        let local_coords = self
                            .primitive
                            .rotation
                            .transform_vector(&(global_coords_point - self.primitive.position));
                        let n = local_coords.component_div(&r);
                        let root = ((n.x * r.y * r.z).powi(2)
                            + (r.x * n.y * r.z).powi(2)
                            + (r.x * r.y * n.z).powi(2))
                        .sqrt();
                        let local_pdf = 1.0 / (4.0 * std::f64::consts::PI * root);
                        local_pdf * (global_coords_point - point).norm_squared()
                            / (intersection.normal.dot(&direction.normalize())).abs()
                    };
                    let mut ans = get_pdf(&r, &first_ray, &intersection);
                    let snd_ray = Ray {
                        origin: point + (intersection.offset + EPS) * direction,
                        direction: *direction,
                    };
                    if let Some(snd_intersection) =
                        intersect_ray_with_primitive(&snd_ray, &self.primitive, f64::INFINITY)
                    {
                        ans += get_pdf(&r, &snd_ray, &snd_intersection);
                    }
                    ans
                }
                Shape3D::Box { s } => {
                    let area = (s.x * s.y + s.y * s.z + s.z * s.x) * 8.0;
                    let local_pdf = 1.0 / area;
                    let global_coords_point1 = point + intersection.offset * direction;

                    let mut ans = local_pdf * (global_coords_point1 - point).norm_squared()
                        / (intersection.normal.dot(&direction.normalize())).abs();
                    if let Some(intersection2) = intersect_ray_with_primitive(
                        &Ray {
                            origin: point + (intersection.offset + EPS) * direction,
                            direction: *direction,
                        },
                        &self.primitive,
                        f64::INFINITY,
                    ) {
                        let global_coords_point2 = point
                            + (intersection.offset + EPS) * direction
                            + intersection2.offset * direction;
                        ans += local_pdf * (global_coords_point2 - point).norm_squared()
                            / (intersection2.normal.dot(&direction.normalize())).abs();
                    }
                    ans
                }
            }
        } else {
            0.0
        }
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
