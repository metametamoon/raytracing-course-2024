use rand::Rng;
use rand_pcg::Pcg32;

use crate::bvh::intersect_with_bvh_all_points;
use crate::distributions::CosineWeightedDistribution;
use crate::distributions::LightSamplingDistribution;
use crate::distributions::MixDistribution;
use crate::distributions::SampleDistribution;
use crate::geometry::Material;
use crate::geometry::Ray;
use crate::geometry::Shape3D;
use crate::geometry::Vec3f;
use crate::geometry::EPS;
use crate::geometry::FP_PI;
use crate::geometry::{get_reflection_ray, Fp};
use crate::geometry::{intersect_ray_with_object3d, Intersection};
use crate::scene::{Primitive, Scene};

type RandGenType = Pcg32;

pub fn render_scene(scene: &Scene) -> Vec<u8> {
    let sample_distribution = MixDistribution {
        distributions: vec![
            Box::new(CosineWeightedDistribution),
            Box::new(MixDistribution {
                distributions: scene
                    .primitives_no_planes
                    .iter()
                    .filter_map(|primitive| {
                        let result: Box<dyn SampleDistribution<Pcg32>> =
                            Box::new(LightSamplingDistribution {
                                object3d: primitive.object3d.clone(),
                            });
                        match primitive.object3d.shape {
                            Shape3D::Ellipsoid { r: _ } => Some(result),
                            Shape3D::Box { s: _ } => Some(result),
                            Shape3D::Triangle { a: _, b: _, c: _ } => Some(result),
                            Shape3D::Plane { norm: _ } => None,
                        }
                    })
                    .collect(),
            }),
        ],
    };

    let mut result = Vec::<u8>::new();
    let mut rng = Pcg32::new(0xcafef00dd15ea5e5, 0xa02bdbf7bb3c0a7);
    for y in 0..scene.height {
        dbg!(y);
        for x in 0..scene.width {
            let color = {
                let total_color = (0..scene.samples)
                    .map(|_: i32| {
                        let ray_to_pixel = get_ray_to_pixel(x, y, scene, &mut rng);
                        get_ray_color(
                            &ray_to_pixel,
                            scene,
                            scene.ray_depth,
                            &sample_distribution,
                            &mut rng,
                        )
                    })
                    .reduce(|x, y| x + y)
                    .unwrap();
                total_color / scene.samples as Fp
            };
            result.extend(color_to_pixel(color));
        }
    }
    result
}

fn get_ray_to_pixel<R: Rng>(x: i32, y: i32, scene: &Scene, rng: &mut R) -> Ray {
    let real_x = x as Fp + rng.gen::<Fp>();
    let real_y = y as Fp + rng.gen::<Fp>();
    let w = scene.width as Fp;
    let h = scene.height as Fp;
    let px = (2.0 * real_x / w - 1.0) * (scene.camera_fov_x * 0.5).tan();
    let py = -(2.0 * real_y / h - 1.0) * (scene.camera_fov_y * 0.5).tan();
    let pz = 1.0 as Fp;
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
    distribution: &dyn SampleDistribution<RandGenType>,
    rng: &mut RandGenType,
) -> Vec3f {
    if recursion_depth <= 0 {
        return Vec3f::default();
    }
    match intersect_ray_with_scene(ray, scene) {
        Some((primitive, intersection)) => match primitive.material {
            Material::Diffused => {
                // if recursion_depth == scene.ray_depth {
                //     if let Shape3D::Triangle { .. } = primitive.object3d.shape {
                //         let _u = 0;
                //         println!("Triangle!");
                //     }
                // }
                let corrected_point = ray.origin + ray.direction * (intersection.offset - EPS);
                let mut total_color = primitive.emission;
                let rnd_vec =
                    distribution.sample_unit_vector(&corrected_point, &intersection.normal, rng);
                let pdf = distribution.pdf(&corrected_point, &intersection.normal, &rnd_vec);
                if pdf > 0.0 && rnd_vec.dot(&intersection.normal) > 0.0 {
                    let color_refl = get_ray_color(
                        &Ray {
                            origin: corrected_point,
                            direction: rnd_vec,
                        },
                        scene,
                        recursion_depth - 1,
                        distribution,
                        rng,
                    );
                    total_color += color_refl.component_mul(&primitive.color)
                        * (1.0 / FP_PI)
                        * rnd_vec.dot(&intersection.normal)
                        * (1.0 / pdf);
                }
                total_color
            }
            Material::Metallic => {
                let reflected_direction = get_reflection_ray(&ray.direction, &intersection.normal);
                let corrected_point = ray.origin + ray.direction * (intersection.offset - EPS);
                let reflection_ray = Ray {
                    origin: corrected_point,
                    direction: reflected_direction,
                };
                get_ray_color(
                    &reflection_ray,
                    scene,
                    recursion_depth - 1,
                    distribution,
                    rng,
                )
                .component_mul(&primitive.color)
            }
            Material::Dielectric => {
                let cosine = -ray.direction.normalize().dot(&intersection.normal);
                let cosine = Fp::min(cosine, 1.0);
                let (eta1, eta2) = if intersection.is_outer_to_inner {
                    (1.0, primitive.ior)
                } else {
                    (primitive.ior, 1.0)
                };
                let sine = (1.0 - cosine.powi(2)).sqrt();
                let sine2 = sine * eta1 / eta2;
                let corrected_point_backward =
                    ray.origin + ray.direction * (intersection.offset - EPS);
                let corrected_point_forward =
                    ray.origin + ray.direction * (intersection.offset + EPS);
                if sine2 > 1.0 {
                    get_reflection_color(
                        ray,
                        scene,
                        recursion_depth,
                        &intersection,
                        corrected_point_backward,
                        distribution,
                        rng,
                    )
                } else {
                    let r0 = ((eta1 - eta2) / (eta1 + eta2)).powi(2);
                    let reflected = r0 + (1.0 - r0) * (1.0 - cosine).powi(5);
                    if rng.gen::<Fp>() < reflected {
                        get_reflection_color(
                            ray,
                            scene,
                            recursion_depth,
                            &intersection,
                            corrected_point_backward,
                            distribution,
                            rng,
                        )
                    } else {
                        let refracted_color = {
                            let cosine2 = (1.0 - sine2 * sine2).sqrt();
                            let new_dir = (eta1 / eta2) * ray.direction.normalize()
                                + (eta1 / eta2 * cosine - cosine2) * intersection.normal;
                            get_ray_color(
                                &Ray {
                                    origin: corrected_point_forward,
                                    direction: new_dir,
                                },
                                scene,
                                recursion_depth - 1,
                                distribution,
                                rng,
                            )
                        };
                        if intersection.is_outer_to_inner {
                            refracted_color.component_mul(&primitive.color)
                        } else {
                            refracted_color
                        }
                    }
                }
            }
        },
        None => scene.bg_color,
    }
}

fn get_reflection_color(
    ray: &Ray,
    scene: &Scene,
    recursion_depth: i32,
    intersection: &Intersection,
    corrected_point: Vec3f,
    distribution: &dyn SampleDistribution<RandGenType>,
    rng: &mut RandGenType,
) -> Vec3f {
    let reflected_direction = get_reflection_ray(&ray.direction, &intersection.normal);
    let reflection_ray = Ray {
        origin: corrected_point,
        direction: reflected_direction,
    };
    get_ray_color(
        &reflection_ray,
        scene,
        recursion_depth - 1,
        distribution,
        rng,
    )
}

fn intersect_ray_with_scene<'a>(
    ray: &Ray,
    scene: &'a Scene,
) -> Option<(&'a Primitive, Intersection)> {
    let mut min_dist = Fp::INFINITY;
    let mut result = None;
    let good = intersect_with_bvh_all_points(
        ray,
        &scene.primitives_no_planes,
        &scene.bvh_nodes,
        &scene.bvh_nodes.len() - 1,
    );
    for (intersection, _ray, primitive) in good {
        if !intersection.is_empty() {
            min_dist = intersection[0].offset;
            result = Some((primitive, intersection[0].clone()))
        }
    }
    for plane_primitive in &scene.infinite_primitives {
        if let Some(intersection) =
            intersect_ray_with_object3d(ray, &plane_primitive.object3d, min_dist)
        {
            min_dist = intersection.offset;
            result = Some((plane_primitive, intersection));
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
