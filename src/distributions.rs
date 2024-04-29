use na::Unit;
use rand::Rng;
use rand_distr::num_traits::Inv;
use rand_distr::{Distribution, Normal};

use crate::bvh::{intersect_with_bvh_all_points, BvhTree};
use crate::geometry::{
    intersect_ray_with_object3d_all_points, Fp, Object3D, Ray, Shape3D, Vec3f, FP_PI,
};
use crate::scene::Material;
use crate::utils::{chi_plus, safe_sqrt};

pub trait SampleDistribution<R: Rng> {
    fn sample_unit_vector(&self, point: &Vec3f, normal: &Vec3f, rng: &mut R) -> Unit<Vec3f>;
    fn pdf(&self, point: &Vec3f, n: &Vec3f, l: &Vec3f, v: &Vec3f, material: &Material) -> Fp;
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
    fn sample_unit_vector(&self, _: &Vec3f, normal: &Vec3f, rng: &mut R) -> Unit<Vec3f> {
        let normal_distr = Normal::new(0.0, 1.0).unwrap();
        let result = Vec3f::new(
            normal_distr.sample(rng),
            normal_distr.sample(rng),
            normal_distr.sample(rng),
        )
        .normalize();
        if result.dot(normal) > 0.0 {
            Unit::<Vec3f>::new_normalize(result)
        } else {
            Unit::<Vec3f>::new_normalize(-result)
        }
    }

    fn pdf(&self, point: &Vec3f, n: &Vec3f, l: &Vec3f, v: &Vec3f, material: &Material) -> Fp {
        1.0 / (2.0 * FP_PI)
    }
}

impl<R: Rng> SampleDistribution<R> for CosineWeightedDistribution {
    fn sample_unit_vector(&self, _point: &Vec3f, normal: &Vec3f, rng: &mut R) -> Unit<Vec3f> {
        let normal_distr = Normal::new(0.0, 1.0).unwrap();
        let uniform = Vec3f::new(
            normal_distr.sample(rng),
            normal_distr.sample(rng),
            normal_distr.sample(rng),
        )
        .normalize();
        Unit::<Vec3f>::new_normalize(uniform + normal)
    }

    fn pdf(&self, point: &Vec3f, n: &Vec3f, l: &Vec3f, v: &Vec3f, material: &Material) -> Fp {
        Fp::max(0.0, l.normalize().dot(n)) / FP_PI
    }
}

fn get_local_pdf(shape: &Shape3D) -> Fp {
    match shape {
        Shape3D::Box { s } => {
            let area = (s.x * s.y + s.y * s.z + s.z * s.x) * 8.0;
            1.0 / area
        }
        Shape3D::Triangle { a, b, c, .. } => {
            let area = (b - a).cross(&(c - a)).norm() * 0.5;
            1.0 / area
        }
    }
}

impl<R: Rng> SampleDistribution<R> for DirectLightSamplingDistribution {
    fn sample_unit_vector(&self, point: &Vec3f, _normal: &Vec3f, rng: &mut R) -> Unit<Vec3f> {
        let local_coords_point = match self.object3d.shape {
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
            Shape3D::Triangle { a, b, c, .. } => {
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

        Unit::<Vec3f>::new_normalize(global_coords_point - point)
    }

    fn pdf(&self, point: &Vec3f, n: &Vec3f, l: &Vec3f, v: &Vec3f, material: &Material) -> Fp {
        let ray = Ray {
            origin: *point,
            direction: *l,
        };
        let (all_intersections, _rotated_ray) =
            intersect_ray_with_object3d_all_points(&ray, &self.object3d);
        let pdf = all_intersections
            .iter()
            .map(|intersection| {
                let global_coords_point = ray.origin + intersection.offset * ray.direction;
                let local_pdf = get_local_pdf(&self.object3d.shape);
                let vector_on_sample = global_coords_point - point;
                let omega = vector_on_sample.normalize();
                local_pdf
                    * (vector_on_sample.norm_squared()
                        / (intersection.normal_geometry.dot(&omega)).abs())
            })
            .sum();
        pdf
    }
}

impl<R: Rng> SampleDistribution<R> for MultipleLightSamplingDistribution {
    fn sample_unit_vector(&self, point: &Vec3f, normal: &Vec3f, rng: &mut R) -> Unit<Vec3f> {
        let n = self.bvh_tree.primitives.len();
        let rand_idx = rng.gen_range(0..n);
        DirectLightSamplingDistribution {
            object3d: self.bvh_tree.primitives[rand_idx].object3d.clone(),
        }
        .sample_unit_vector(point, normal, rng)
    }

    fn pdf(&self, point: &Vec3f, n: &Vec3f, l: &Vec3f, v: &Vec3f, material: &Material) -> Fp {
        let ray = Ray {
            origin: *point,
            direction: *l,
        };
        let bvh_intersections = intersect_with_bvh_all_points(&ray, &self.bvh_tree);
        let pdf = bvh_intersections
            .iter()
            .map(|bvh_intersection| {
                let object3d = &bvh_intersection.primitive.object3d;
                let mut sum = 0.0;
                for intersection in &bvh_intersection.intersections {
                    let global_coords_point = ray.origin + intersection.offset * ray.direction;
                    let local_pdf = get_local_pdf(&object3d.shape);
                    let vector_on_sample = global_coords_point - point;
                    let omega = vector_on_sample.normalize();
                    sum += local_pdf
                        * (vector_on_sample.norm_squared()
                            / (intersection.normal_geometry.dot(&omega)).abs())
                }
                sum
            })
            .sum::<Fp>();
        pdf / (self.bvh_tree.primitives.len() as Fp)
    }
}

impl<R: Rng> SampleDistribution<R> for MixDistribution<R> {
    fn sample_unit_vector(&self, point: &Vec3f, normal: &Vec3f, rng: &mut R) -> Unit<Vec3f> {
        let n = self.distributions.len();
        let rand_idx = rng.gen_range(0..n);
        self.distributions[rand_idx].sample_unit_vector(point, normal, rng)
    }

    fn pdf(&self, point: &Vec3f, n: &Vec3f, l: &Vec3f, v: &Vec3f, material: &Material) -> Fp {
        let ans = self
            .distributions
            .iter()
            .map(|distr| distr.pdf(point, n, l, v, material))
            .sum::<Fp>();
        ans / (self.distributions.len() as Fp)
    }
}

struct VndfDistribution {}

impl VndfDistribution {
    fn pdf(&self, point: &Vec3f, normal: &Vec3f, direction: &Vec3f, material: &Material) -> Fp {
        let alpha: Fp = material.metallic_roughness.powi(2); // roughness^2
        let h = normal;
        let n = normal;
        let l = direction;
        let v = todo!();
        let d = {
            let hn = h.dot(n);
            let numerator = alpha.powi(2) * chi_plus(hn);
            let denominator = FP_PI * ((alpha.powi(2) - 1.0) * hn * hn + 1.0).powi(2);
            numerator / denominator
        };
        let g1 = |x: &Vec3f| {
            let nx = n.dot(x);
            let a = {
                let numerator = nx * chi_plus(nx);
                let denominator = alpha * safe_sqrt(1.0 - nx.powi(2));
                numerator / denominator
            };
            let under_sqrt = 1.0 + 1.0 / (a * a);
            let lambda = 0.5 * (under_sqrt.sqrt() - 1.0);
            1.0 / (1.0 + lambda)
        };

        // let g = {
        //     g1(n, l) * g1(n, v)
        // };
        // Dv(N) / (4 * (V dot N))
        // Dv(N) = G1(V) * max(0, V * N) * D(N) / * (V * Z)
        0.0
    }

    fn sample_ggx_vndf<R: Rng>(Ve: Vec3f, alpha: Fp, rng: &mut R) -> Vec3f {
        let U1 = rng.gen_range(0.0..1.0);
        let U2 = rng.gen_range(0.0..1.0);
        // Section 3.2: transforming the view direction to the hemisphere configuration
        let Vh = (Vec3f::new(alpha * Ve.x, alpha * Ve.y, Ve.z)).normalize();
        // Section 4.1: orthonormal basis (with special case if cross product is zero)
        let lensq = Vh.x * Vh.x + Vh.y * Vh.y;
        let T1 = if lensq > 0.0 {
            Vec3f::new(-Vh.y, Vh.x, 0.0) * lensq.inv().sqrt()
        } else {
            Vec3f::new(1.0, 0.0, 0.0)
        };
        let T2 = Vh.cross(&T1);
        // Section 4.2: parameterization of the projected area
        let r = Fp::sqrt(U1);
        let phi = 2.0 * FP_PI * U2;
        let t1 = r * Fp::cos(phi);
        let t2 = r * Fp::sin(phi);
        let s = 0.5 * (1.0 + Vh.z);
        let t2 = (1.0 - s) * Fp::sqrt(1.0 - t1 * t1) + s * t2;
        // Section 4.3: reprojection onto hemisphere
        let Nh = t1 * T1 + t2 * T2 + Fp::sqrt(Fp::max(0.0, 1.0 - t1 * t1 - t2 * t2)) * Vh;
        // Section 3.4: transforming the normal back to the ellipsoid configuration
        let Ne = (Vec3f::new(alpha * Nh.x, alpha * Nh.y, Fp::max(0.0, Nh.z))).normalize();
        Ne
    }
}
