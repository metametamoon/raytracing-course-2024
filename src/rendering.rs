use std::f64::consts::PI;

use crate::distributions::DirectionOnObjectDistribution;
use crate::distributions::MixDistribution;
use crate::distributions::SampleDistribution;
use crate::distributions::{CosineWeightedDistribution, SemisphereUniform};
use crate::geometry::get_reflection_ray;
use crate::geometry::intersect_ray_with_primitive;
use crate::geometry::Intersection;
use crate::geometry::Material;
use crate::geometry::Primitive;
use crate::geometry::Ray;
use crate::geometry::Shape3D;
use crate::geometry::Vec3f;
use crate::geometry::EPS;
use crate::scene::Scene;
use rand_distr::{Distribution, Normal};
use rayon::prelude::*;

static PARALLEL: bool = false;

pub fn render_scene(scene: &Scene) -> Vec<u8> {
    let sample_distribution = MixDistribution {
        distributions: vec![
            Box::new(CosineWeightedDistribution),
            Box::new(MixDistribution {
                distributions: scene
                    .primitives
                    .iter()
                    .filter_map(|x| {
                        let result: Box<dyn SampleDistribution> =
                            Box::new(DirectionOnObjectDistribution {
                                primitive: x.clone(),
                            });
                        match x.shape {
                            Shape3D::None => None,
                            Shape3D::Plane { norm } => None,
                            Shape3D::Ellipsoid { r } => Some(result),
                            Shape3D::Box { s } => Some(result),
                        }
                    })
                    .collect(),
            }),
        ],
    };
    let sample_distribution = CosineWeightedDistribution;

    let mut result = Vec::<u8>::new();
    for y in 0..scene.height {
        // println!("{}", y);
        for x in 0..scene.width {
            let ray_to_pixel = get_ray_to_pixel(x, y, scene);
            let color = {
                // parallel:
                // let total_color=
                //     (0..scene.samples)
                //         .into_par_iter()
                //         .map(|_: i32| {
                //             get_ray_color(
                //                 &ray_to_pixel,
                //                 scene,
                //                 scene.ray_depth,
                //                 &sample_distribution,
                //             )
                //         })
                //         .reduce(|| Vec3f::default(), |x, y| x + y);
                let total_color = (0..scene.samples)
                    .map(|_: i32| {
                        get_ray_color(&ray_to_pixel, scene, scene.ray_depth, &sample_distribution)
                    })
                    .reduce(|x, y| x + y)
                    .unwrap();
                total_color / scene.samples as f64
            };
            result.extend(color_to_pixel(color));
        }
    }
    result
}

fn get_ray_to_pixel(x: i32, y: i32, scene: &Scene) -> Ray {
    let real_x = x as f64 + 0.5;
    let real_y = y as f64 + 0.5;
    let w = scene.width as f64;
    let h = scene.height as f64;
    let px = (2.0 * real_x / w - 1.0) * (scene.camera_fov_x * 0.5).tan();
    let py = -(2.0 * real_y / h - 1.0) * (scene.camera_fov_y * 0.5).tan();
    let pz = 1.0f64;
    let direction = px * scene.camera_right + py * scene.camera_up + pz * scene.camera_forward;
    Ray {
        origin: scene.camera_position,
        direction,
    }
}

fn get_ray_color(
    ray: &Ray,
    scene: &Scene,
    recursion_depth: i32,
    distribution: &dyn SampleDistribution,
) -> Vec3f {
    if recursion_depth <= 0 {
        return Vec3f::default();
    }
    match intersect_ray_with_scene(ray, scene) {
        Some((primitive, intersection)) => {
            let intersection_point = ray.origin + ray.direction * intersection.offset;
            match primitive.material {
                Material::Diffused => {
                    let mut total_color = primitive.emission;
                    // let distribution = CosineWeightedDistribution;
                    let (rnd_vec, pdf) = loop {
                        let rnd_vec =
                            distribution.sample(&intersection_point, &intersection.normal);
                        let pdf =
                            distribution.pdf(&intersection_point, &intersection.normal, &rnd_vec);
                        if pdf > 0.0 {
                            break (rnd_vec, pdf);
                        } else {
                            log::info!("Failed a sampling (probably a floating-point error)");
                        }
                    };
                    assert!(pdf > 0.0, "pdf should be greated then zero!");
                    let color_refl = get_ray_color(
                        &Ray {
                            origin: intersection_point + EPS * intersection.normal,
                            direction: rnd_vec,
                        },
                        scene,
                        recursion_depth - 1,
                        distribution,
                    );
                    total_color += color_refl.component_mul(&primitive.color)
                        * std::f64::consts::FRAC_1_PI
                        * rnd_vec.dot(&intersection.normal)
                        * (1.0 / pdf);
                    total_color
                }
                Material::Metallic => {
                    let reflected_direction =
                        get_reflection_ray(&ray.direction, &intersection.normal);
                    let corrected_point = intersection_point + intersection.normal * EPS;
                    let reflection_ray = Ray {
                        origin: corrected_point,
                        direction: reflected_direction,
                    };
                    get_ray_color(&reflection_ray, scene, recursion_depth - 1, distribution)
                        .component_mul(&primitive.color)
                }
                Material::Dielectric => {
                    let cosine = -ray.direction.normalize().dot(&intersection.normal);
                    let cosine = f64::min(cosine, 1.0);
                    let (eta1, eta2) = if intersection.is_outer_to_inner {
                        (1.0, primitive.ior)
                    } else {
                        (primitive.ior, 1.0)
                    };
                    let sine = (1.0 - cosine.powi(2)).sqrt();
                    let sine2 = sine * eta1 / eta2;
                    if sine2 > 1.0 {
                        get_reflection_color(
                            &ray,
                            scene,
                            recursion_depth,
                            &intersection,
                            intersection_point,
                            distribution,
                        )
                    } else {
                        let r0 = ((eta1 - eta2) / (eta1 + eta2)).powi(2);
                        let reflected = r0 + (1.0 - r0) * (1.0 - cosine).powi(5);
                        if fastrand::f64() < reflected {
                            let reflection_color = get_reflection_color(
                                &ray,
                                scene,
                                recursion_depth,
                                &intersection,
                                intersection_point,
                                distribution,
                            );
                            reflection_color
                        } else {
                            let refracted_color = {
                                let outer_norm = -intersection.normal;
                                let outer_norm = outer_norm * ray.direction.dot(&outer_norm);
                                let tan2 = sine2 / (1.0 - sine2 * sine2).sqrt();
                                if (ray.direction - outer_norm).norm() < EPS {
                                    let corrected_point =
                                        intersection_point - intersection.normal * EPS;
                                    get_ray_color(
                                        &Ray {
                                            origin: corrected_point,
                                            direction: ray.direction,
                                        },
                                        scene,
                                        recursion_depth - 1,
                                        distribution,
                                    )
                                } else {
                                    let v2 = (ray.direction - outer_norm).normalize();
                                    let norm_len = outer_norm.norm();
                                    let refracted_dir = outer_norm + norm_len * tan2 * v2;
                                    let corrected_point =
                                        intersection_point - intersection.normal * EPS;
                                    get_ray_color(
                                        &Ray {
                                            origin: corrected_point,
                                            direction: refracted_dir,
                                        },
                                        scene,
                                        recursion_depth - 1,
                                        distribution,
                                    )
                                }
                            };
                            if intersection.is_outer_to_inner {
                                refracted_color.component_mul(&primitive.color)
                            } else {
                                refracted_color
                            }
                        }
                    }
                }
            }
        }
        None => scene.bg_color,
    }
}

fn get_reflection_color(
    ray: &Ray,
    scene: &Scene,
    recursion_depth: i32,
    intersection: &Intersection,
    intersection_point: Vec3f,
    distribution: &dyn SampleDistribution,
) -> Vec3f {
    let reflected_direction = get_reflection_ray(&ray.direction, &intersection.normal);
    let corrected_point = intersection_point + intersection.normal * EPS;
    let reflection_ray = Ray {
        origin: corrected_point,
        direction: reflected_direction,
    };
    get_ray_color(&reflection_ray, scene, recursion_depth - 1, distribution)
}

fn intersect_ray_with_scene<'a>(
    ray: &Ray,
    scene: &'a Scene,
) -> Option<(&'a Primitive, Intersection)> {
    let mut min_dist = f64::INFINITY;
    let mut result = None;
    for primitive in &scene.primitives {
        if let Some(intersection) = intersect_ray_with_primitive(&ray, &primitive, min_dist) {
            min_dist = intersection.offset;
            let mut corrected_intersection = intersection;
            corrected_intersection.normal = primitive
                .rotation
                .transform_vector(&corrected_intersection.normal);
            result = Some((primitive, corrected_intersection))
        }
    }
    result
}

fn saturate(color: Vec3f) -> Vec3f {
    Vec3f::new(
        color.x.clamp(0.0, 1.0),
        color.y.clamp(0.0, 1.0),
        color.z.clamp(0.0, 1.0),
    )
}

fn aces_tonemap(x: Vec3f) -> Vec3f {
    let a = 2.51;
    let b = 0.03;
    let c = 2.43;
    let d = 0.59;
    let e = 0.14;
    let unit = Vec3f::new(1.0, 1.0, 1.0);

    saturate(
        (x.component_mul(&(a * x + b * unit)))
            .component_div(&(x.component_mul(&(c * x + d * unit)) + e * unit)),
    )
}

fn color_to_pixel(color: Vec3f) -> [u8; 3] {
    let tonemapped = aces_tonemap(color);
    let gamma_corrected = Vec3f::new(
        tonemapped.x.powf(1.0 / 2.2),
        tonemapped.y.powf(1.0 / 2.2),
        tonemapped.z.powf(1.0 / 2.2),
    );
    [
        (gamma_corrected.x * 255.0).round() as u8,
        (gamma_corrected.y * 255.0).round() as u8,
        (gamma_corrected.z * 255.0).round() as u8,
    ]
}

fn random_vector_norm(normal: &Vec3f) -> Vec3f {
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

fn random_vector_loop(normal: &Vec3f) -> Vec3f {
    loop {
        let v = Vec3f::new(
            fastrand::f64() * 2.0 - 1.0,
            fastrand::f64() * 2.0 - 1.0,
            fastrand::f64() * 2.0 - 1.0,
        );
        let norm = v.norm();
        if norm <= 1.0 {
            let result = v.unscale(norm);
            if result.dot(&normal) > 0.0 {
                return result;
            } else {
                return -result;
            }
        }
    }
}

// the worst ever algo
fn random_vector_trigonometry(normal: &Vec3f) -> Vec3f {
    let theta: f64 = (fastrand::f64() * 2.0 - 1.0).acos();
    let phi: f64 = 2.0 * PI * fastrand::f64();
    let x = theta.sin() * phi.cos();
    let y = theta.sin() * phi.sin();
    let z = theta.cos();
    let result = Vec3f::new(x, y, z);
    if result.dot(&normal) > 0.0 {
        result
    } else {
        -result
    }
}
