use indicatif::{ParallelProgressIterator, ProgressBar};
use rand::{Rng, RngCore, thread_rng};
use rand_xoshiro::rand_core::SeedableRng;
use rand_xoshiro::Xoshiro256StarStar;
use rayon::iter::ParallelIterator;
use rayon::prelude::IntoParallelRefIterator;

use crate::bvh::{interesect_with_bvh_nearest_point, validate_bvh};
use crate::distributions::MixDistribution;
use crate::distributions::SampleDistribution;
use crate::distributions::{CosineWeightedDistribution, MultipleLightSamplingDistribution};
use crate::geometry::Ray;
use crate::geometry::Vec3f;
use crate::geometry::EPS;
use crate::geometry::FP_PI;
use crate::geometry::{get_reflection_ray, Fp};
use crate::geometry::{intersect_ray_with_object3d, Intersection};
use crate::scene::{Material, MaterialEnumerated, Primitive, Scene};

pub fn render_scene(scene: &Scene) -> Vec<u8> {
    validate_bvh(&scene.bvh_finite_primitives);
    let mut distributions: Vec<Box<dyn SampleDistribution<_> + Sync>> =
        vec![Box::new(CosineWeightedDistribution)];
    if !scene.bvh_light_sources.primitives.is_empty() {
        distributions.push(Box::new(MultipleLightSamplingDistribution {
            bvh_tree: scene.bvh_light_sources.clone(),
        }))
    }

    let sample_distribution = MixDistribution { distributions };

    let sample_distr = &sample_distribution;
    // {
    //     let mut rng: Xoshiro256StarStar =
    //         Xoshiro256StarStar::seed_from_u64(0u64);
    //     let ray_to_pixel = get_ray_to_pixel(20, 200, scene, &mut rng);
    //     let color = get_ray_color(&ray_to_pixel, scene, scene.ray_depth, sample_distr, &mut rng);
    //     println!("My color:{}", color);
    // }
    let result = (0..scene.height)
        .collect::<Vec<_>>()
        .par_iter()
        .progress_count(scene.height as u64)
        .flat_map_iter(|y| {
            // dbg!(y);
            let dist = sample_distr;
            let mut rng: Xoshiro256StarStar =
                Xoshiro256StarStar::seed_from_u64((scene.width * y) as u64);
            (0..scene.width).flat_map(move |x| {
                let color = {
                    
                    let total_color = (0..scene.samples)
                        .map(|_: i32| {
                            let ray_to_pixel = get_ray_to_pixel(x, *y, scene, &mut rng);
                            get_ray_color(&ray_to_pixel, scene, scene.ray_depth, dist, &mut rng)
                        })
                        .reduce(|x, y| x + y)
                        .unwrap();
                    total_color / scene.samples as Fp
                };
                color_to_pixel(color)
            })
        })
        .collect::<Vec<_>>();
    // bar.finish();
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

fn chi_plus(x: Fp) -> Fp {
    if x > 0.0 {
        x
    } else {
        0.0
    }
}

fn safe_sqrt(x: Fp) -> Fp {
    Fp::max(0.0, x).sqrt()
}

fn get_ray_color<RandGenType: RngCore>(
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
        Some((primitive, intersection)) => {
            let corrected_point = ray.origin + ray.direction * (intersection.offset - EPS);
            let mut total_color = primitive.emission;
            let rnd_vec = distribution
                .sample_unit_vector(&corrected_point, &intersection.normal_shading, rng)
                .into_inner();
            let pdf = distribution.pdf(&corrected_point, &intersection.normal_shading, &rnd_vec);
            if pdf > 0.0 && rnd_vec.dot(&intersection.normal_geometry) > 0.0 {
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
                // if recursion_depth == 6 {
                //     println!("Dbg");
                // }
                let brdf = brdf(
                    &-ray.direction,
                    &intersection.normal_shading,
                    &rnd_vec,
                    &primitive.material,
                );
                total_color += color_refl.component_mul(&brdf)
                    * rnd_vec.dot(&intersection.normal_shading)
                    * (1.0 / pdf);
            }
            total_color
        }
        None => scene.bg_color,
    }
}

fn fresnel_term(f0: &Vec3f, f90: &Vec3f, h: &Vec3f, l: &Vec3f) -> Vec3f {
    f0 + (f90 - f0) * (1.0 - h.dot(l)).powi(5)
}

fn brdf(l: &Vec3f, n: &Vec3f, v: &Vec3f, material: &Material) -> Vec3f {
    let h = (l + v).normalize();
    let diffuse_brdf = material.base_color_factor / FP_PI;
    let specular_brdf = specular_brdf(l, n, v, &h, material);

    let metal_brdf = specular_brdf
        * fresnel_term(
            &material.base_color_factor,
            &Vec3f::from_element(1.0),
            &h,
            l,
        );
    let dielectric_brdf = {
        let f = fresnel_term(&Vec3f::from_element(0.04), &Vec3f::from_element(1.0), &h, l);
        specular_brdf * f + diffuse_brdf.component_mul(&(Vec3f::from_element(1.0) - f))
    };
    metal_brdf * material.metallic_factor + dielectric_brdf * (1.0 - material.metallic_factor)
}

fn specular_brdf(l: &Vec3f, n: &Vec3f, v: &Vec3f, h: &Vec3f, material: &Material) -> Fp {
    let alpha: Fp = material.metallic_roughness.powi(2); // roughness^2
    let d = {
        let hn = h.dot(n);
        let numerator = alpha.powi(2) * chi_plus(hn);
        let denominator = FP_PI * ((alpha.powi(2) - 1.0) * hn * hn + 1.0).powi(2);
        numerator / denominator
    };
    let g = {
        let g1 = |n: &Vec3f, x: &Vec3f| {
            let nx = n.dot(x);
            let a = {
                let numerator = nx * chi_plus(nx);
                let denominator = alpha * safe_sqrt(1.0 - nx.powi(2));
                numerator / denominator
            };
            let under_sqrt = (1.0 + 1.0 / (a * a));
            let lambda = 0.5 * (under_sqrt.sqrt() - 1.0);
            1.0 / (1.0 + lambda)
        };
        g1(n, l) * g1(n, v)
    };
    let component = d * g / (4.0 * (l.dot(n)) * (v.dot(n)));
    component
}

fn get_reflection_color<RandGenType: RngCore>(
    ray: &Ray,
    scene: &Scene,
    recursion_depth: i32,
    intersection: &Intersection,
    corrected_point: Vec3f,
    distribution: &dyn SampleDistribution<RandGenType>,
    rng: &mut RandGenType,
) -> Vec3f {
    let reflected_direction = get_reflection_ray(&ray.direction, &intersection.normal_geometry);
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
    let mut latest_intersection_offset = Fp::INFINITY;
    let mut result = None;
    let maybe_intersection = interesect_with_bvh_nearest_point(ray, &scene.bvh_finite_primitives);
    if let Some(bvh_intersection) = maybe_intersection {
        latest_intersection_offset = bvh_intersection.intersections[0].offset;
        result = Some((
            bvh_intersection.primitive,
            bvh_intersection.intersections[0].clone(),
        ))
    }
    for plane_primitive in &scene.infinite_primitives {
        if let Some(intersection) =
            intersect_ray_with_object3d(ray, &plane_primitive.object3d, latest_intersection_offset)
        {
            if intersection.offset < latest_intersection_offset {
                latest_intersection_offset = intersection.offset;
                result = Some((plane_primitive, intersection));
            }
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
