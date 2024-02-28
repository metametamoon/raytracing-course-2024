use crate::scene::LightLocation;
use crate::scene::Material;
use crate::scene::Primitive;
use crate::scene::Scene;
use crate::scene::Shape3D;
use crate::scene::Vec3f;
use na::Vector3;
use rand::Rng;

#[derive(Clone)]
struct Ray {
    origin: Vec3f,
    direction: Vec3f,
}

#[derive(Default, Clone)]
struct Intersection {
    pub offset: f64,
    pub normal: Vec3f,
    pub is_outer_to_inner: bool,
}

static EPS: f64 = 0.0001;

fn intersect(ray: Ray, shape: &Shape3D) -> Vec<Intersection> {
    match shape {
        Shape3D::None => {
            panic!("Intersect with None")
        }
        Shape3D::Plane { norm } => {
            let x = norm.dot(&ray.direction);
            if x == 0.0 {
                vec![]
            } else {
                vec![Intersection {
                    offset: -ray.origin.dot(norm) / x,
                    normal: if x < 0.0 {
                        norm.normalize()
                    } else {
                        -norm.normalize()
                    },
                    is_outer_to_inner: true,
                }]
            }
        }
        Shape3D::Ellipsoid { rx, ry, rz } => {
            let r = Vector3::new(*rx, *ry, *rz);
            let d1 = ray.direction.component_div(&r);
            let o1 = ray.origin.component_div(&r);
            let a = d1.dot(&d1);
            let b = 2.0 * o1.dot(&d1);
            let c = o1.dot(&o1) - 1.0;
            let discr = b * b - 4.0 * a * c;
            let x1 = (-b - discr.sqrt()) / (2.0 * a);
            let x2 = (-b + discr.sqrt()) / (2.0 * a);

            let t1 = f64::min(x1, x2);
            let t2 = f64::max(x1, x2);
            let p1 = ray.origin + ray.direction * t1;
            let p2 = ray.origin + ray.direction * t2;

            let norm1 =
                Vec3f::new(p1.x / (rx * rx), p1.y / (ry * ry), p1.z / (rz * rz)).normalize();

            let norm2 =
                Vec3f::new(p2.x / (rx * rx), p2.y / (ry * ry), p2.z / (rz * rz)).normalize();

            if discr >= 0.0 {
                vec![
                    Intersection {
                        offset: t1,
                        normal: norm1,
                        is_outer_to_inner: true,
                    },
                    Intersection {
                        offset: t2,
                        normal: -norm2,
                        is_outer_to_inner: false,
                    },
                ]
            } else {
                vec![]
            }
        }
        Shape3D::Box { sx, sy, sz } => {
            let mut t_x = [
                (-sx - ray.origin.x) / ray.direction.x,
                (sx - ray.origin.x) / ray.direction.x,
            ];
            t_x.sort_by(|a, b| a.partial_cmp(b).unwrap());

            let mut t_y = [
                (-sy - ray.origin.y) / ray.direction.y,
                (sy - ray.origin.y) / ray.direction.y,
            ];
            t_y.sort_by(|a, b| a.partial_cmp(b).unwrap());

            let mut t_z = [
                (-sz - ray.origin.z) / ray.direction.z,
                (sz - ray.origin.z) / ray.direction.z,
            ];
            t_z.sort_by(|a, b| a.partial_cmp(b).unwrap());

            let t_min = f64::max(t_x[0], f64::max(t_y[0], t_z[0]));
            let t_max = f64::min(t_x[1], f64::min(t_y[1], t_z[1]));
            if t_min < t_max {
                let p_min = ray.origin + ray.direction * t_min;
                let p_max = ray.origin + ray.direction * t_max;
                let s = Vector3::new(*sx, *sy, *sz);
                let norm_min = {
                    let mut tmp = p_min.component_div(&s);
                    let max_coord = f64::max(tmp.x.abs(), f64::max(tmp.y.abs(), tmp.z.abs()));
                    if tmp.x.abs() < max_coord {
                        tmp.x = 0.0
                    }
                    if tmp.y.abs() < max_coord {
                        tmp.y = 0.0
                    }
                    if tmp.z.abs() < max_coord {
                        tmp.z = 0.0
                    }
                    if tmp.norm() < 1.5 {
                        tmp
                    } else {
                        tmp.x = 0.0;
                        if tmp.norm() < 1.5 {
                            tmp
                        } else {
                            tmp.y = 0.0;
                            tmp
                        }
                    }
                };
                let norm_max = {
                    let mut tmp = p_max.component_div(&s);
                    let max_coord = f64::max(tmp.x.abs(), f64::max(tmp.y.abs(), tmp.z.abs()));
                    if tmp.x.abs() < max_coord {
                        tmp.x = 0.0
                    }
                    if tmp.y.abs() < max_coord {
                        tmp.y = 0.0
                    }
                    if tmp.z.abs() < max_coord {
                        tmp.z = 0.0
                    }
                    if tmp.norm() < 1.5 {
                        tmp
                    } else {
                        tmp.x = 0.0;
                        if tmp.norm() < 1.5 {
                            tmp
                        } else {
                            tmp.y = 0.0;
                            tmp
                        }
                    }
                };
                // if !(norm_min.norm() + EPS > 1.0) || !(norm_min.norm() - EPS < 1.0) {
                //     println!(
                //         "after div: {:?}, before div: {:?}",
                //         norm_min,
                //         p_min.component_div(&s)
                //     );
                // }

                // if !(norm_max.norm() - EPS < 1.0) || !(norm_max.norm() + EPS > 1.0) {
                //     println!(
                //         "after div: {:?}, before div: {:?}",
                //         norm_max,
                //         p_max.component_div(&s)
                //     );
                // }
                vec![
                    Intersection {
                        offset: t_min,
                        normal: norm_min.normalize(),
                        is_outer_to_inner: true,
                    },
                    Intersection {
                        offset: t_max,
                        normal: -norm_max.normalize(),
                        is_outer_to_inner: false,
                    },
                ]
            } else {
                vec![]
            }
        }
    }
}

pub fn render_scene(scene: &Scene) -> Vec<u8> {
    let mut result = Vec::<u8>::new();
    for y in 0..scene.height {
        // if y % 100 == 0 {
        //     println!("y={}", y);
        // }
        for x in 0..scene.width {
            let ray_to_pixel = get_ray_to_pixel(x, y, scene);
            let color = {
                let mut color = Vec3f::default();
                for _ in 0..scene.samples {
                    color += get_ray_color(ray_to_pixel.clone(), scene, scene.ray_depth);
                }
                color / scene.samples as f64
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

fn get_ray_color(ray: Ray, scene: &Scene, recursion_depth: i32) -> Vec3f {
    if recursion_depth <= 0 {
        return Vec3f::default();
    }
    match intersect_ray_with_scene(ray.clone(), scene) {
        Some((primitive, intersection)) => {
            let intersection_point = ray.origin + ray.direction * intersection.offset;
            match primitive.material {
                Material::Diffused => {
                    let mut total_color = primitive.emission;
                    let rnd_vec = random_vector(&intersection.normal);
                    let color_refl = get_ray_color(
                        Ray {
                            origin: intersection_point + EPS * intersection.normal,
                            direction: rnd_vec,
                        },
                        scene,
                        recursion_depth - 1,
                    );
                    total_color += color_refl.component_mul(&primitive.color)
                        * rnd_vec.dot(&intersection.normal)
                        * 2.0;
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
                    get_ray_color(reflection_ray, scene, recursion_depth - 1)
                        .component_mul(&primitive.color)
                }
                // _ => primitive.color,
                Material::Dielectric => {
                    let cosine = -ray.direction.normalize().dot(&intersection.normal);
                    if cosine < 0.0 {
                        println!("cos={}", cosine);
                    }
                    let cosine = cosine.abs().clamp(0.0, 1.0);
                    assert!(cosine >= 0.0);
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
                        )
                    } else {
                        let r0 = ((eta1 - eta2) / (eta1 + eta2)).powi(2);
                        let reflected = r0 + (1.0 - r0) * (1.0 - cosine).powi(5);
                        let mut rng = rand::thread_rng();
                        if rng.gen::<f64>() < reflected {
                            let reflection_color = get_reflection_color(
                                &ray,
                                scene,
                                recursion_depth,
                                &intersection,
                                intersection_point,
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
                                        Ray {
                                            origin: corrected_point,
                                            direction: ray.direction,
                                        },
                                        scene,
                                        recursion_depth - 1,
                                    )
                                } else {
                                    let v2 = (ray.direction - outer_norm).normalize();
                                    let norm_len = outer_norm.norm();
                                    let refracted_dir = outer_norm + norm_len * tan2 * v2;
                                    let corrected_point =
                                        intersection_point - intersection.normal * EPS;
                                    get_ray_color(
                                        Ray {
                                            origin: corrected_point,
                                            direction: refracted_dir,
                                        },
                                        scene,
                                        recursion_depth - 1,
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
                } // _ => primitive.color,
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
) -> Vec3f {
    let reflected_direction = get_reflection_ray(&ray.direction, &intersection.normal);
    let corrected_point = intersection_point + intersection.normal * EPS;
    let reflection_ray = Ray {
        origin: corrected_point,
        direction: reflected_direction,
    };
    get_ray_color(reflection_ray, scene, recursion_depth - 1)
}

fn get_reflection_ray(ray: &Vec3f, normal: &Vec3f) -> Vec3f {
    let projection = ray.dot(normal).abs();
    ray + normal * projection * 2.0
}

fn intersect_ray_with_scene(ray: Ray, scene: &Scene) -> Option<(Primitive, Intersection)> {
    scene
        .primitives
        .iter()
        .flat_map(|primitive| {
            let transposed_ray = Ray {
                origin: ray.origin - primitive.position,
                direction: ray.direction,
            };
            let rotated_ray = Ray {
                origin: primitive
                    .rotation
                    .conjugate()
                    .transform_vector(&transposed_ray.origin),
                direction: primitive
                    .rotation
                    .conjugate()
                    .transform_vector(&transposed_ray.direction),
            };

            intersect(rotated_ray, &primitive.shape)
                .iter()
                .map(|t| {
                    (
                        primitive.clone(),
                        Intersection {
                            offset: t.offset,
                            normal: primitive.rotation.transform_vector(&t.normal),
                            is_outer_to_inner: t.is_outer_to_inner,
                        },
                    )
                })
                .collect::<Vec<(Primitive, Intersection)>>()
        })
        .filter(|(_, intersection)| {
            intersection.offset >= 0.0 && intersection.offset != f64::INFINITY
        })
        .min_by(|a, b| a.1.offset.partial_cmp(&b.1.offset).unwrap())
}

fn saturate(color: Vector3<f64>) -> Vector3<f64> {
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

fn random_vector(normal: &Vec3f) -> Vec3f {
    let mut rng = rand::thread_rng();
    loop {
        let v = Vec3f::new(
            rng.gen_range(-1.0..1.0),
            rng.gen_range(-1.0..1.0),
            rng.gen_range(-1.0..1.0),
        );
        if v.norm() <= 1.0 && v.dot(&normal) > 0.0 {
            return v.normalize();
        }
    }
}
