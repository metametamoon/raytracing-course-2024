use crate::scene::Primitive;
use crate::scene::Scene;
use crate::scene::Shape3D;
use crate::scene::Vec3f;
use na::Vector3;

struct Ray {
    origin: Vec3f,
    direction: Vec3f,
}

#[derive(Default, Clone)]
struct Intersection {
    pub offset: f32,
    pub normal: Vec3f,
    pub is_outer_to_inner: bool,
}

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
                    normal: *norm,
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
            let p1 = ray.origin + ray.direction * x1;
            let p2 = ray.origin + ray.direction * x2;

            let norm1: Vec3f = p1.component_div(&r);
            let norm1 = norm1.component_div(&r);

            let norm2: Vec3f = p2.component_div(&r);
            let norm2 = norm2.component_div(&r);

            if discr >= 0.0 {
                vec![
                    Intersection {
                        offset: f32::min(x1, x2),
                        normal: norm1,
                        is_outer_to_inner: true,
                    },
                    Intersection {
                        offset: f32::max(x1, x2),
                        normal: norm2,
                        is_outer_to_inner: false,
                    },
                ]
            } else {
                vec![]
            }
        }
        Shape3D::Box { sx, sy, sz } => {
            if ray.direction.x == 0.0 || ray.direction.y == 0.0 || ray.direction.z == 0.0 {
                panic!("I pray this is unreachable")
            } else {
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

                let t_min = f32::max(t_x[0], f32::max(t_y[0], t_z[0]));
                let t_max = f32::min(t_x[1], f32::min(t_y[1], t_z[1]));
                if t_min < t_max {
                    let p_min = ray.origin + ray.direction * t_min;
                    let p_max = ray.origin + ray.direction * t_max;
                    let s = Vector3::new(*sx, *sy, *sz);
                    let norm_min = {
                        let mut tmp = p_min.component_div(&s);
                        if tmp.x.abs() < 1.0 {
                            tmp.x = 0.0
                        }
                        if tmp.y.abs() < 1.0 {
                            tmp.y = 0.0
                        }
                        if tmp.z.abs() < 1.0 {
                            tmp.z = 0.0
                        }
                        tmp
                    };
                    let norm_max = {
                        let mut tmp = p_max.component_div(&s);
                        if tmp.x.abs() < 1.0 {
                            tmp.x = 0.0
                        }
                        if tmp.y.abs() < 1.0 {
                            tmp.y = 0.0
                        }
                        if tmp.z.abs() < 1.0 {
                            tmp.z = 0.0
                        }
                        tmp
                    };
                    vec![
                        Intersection {
                            offset: t_min,
                            normal: norm_min,
                            is_outer_to_inner: true,
                        },
                        Intersection {
                            offset: t_max,
                            normal: norm_max,
                            is_outer_to_inner: false,
                        },
                    ]
                } else {
                    vec![]
                }
            }
        }
    }
}

pub fn render_scene(scene: &Scene) -> Vec<u8> {
    let mut result = Vec::<u8>::new();
    for y in 0..scene.height {
        for x in 0..scene.width {
            let ray_to_pixel = get_ray_to_pixel(x, y, scene);
            let color = get_ray_color(ray_to_pixel, scene);
            result.push((color.x * 255.0).round() as u8);
            result.push((color.y * 255.0).round() as u8);
            result.push((color.z * 255.0).round() as u8);
        }
    }
    result
}

fn get_ray_to_pixel(x: i32, y: i32, scene: &Scene) -> Ray {
    let real_x = x as f32 + 0.5;
    let real_y = y as f32 + 0.5;
    let w = scene.width as f32;
    let h = scene.height as f32;
    let px = (2.0 * real_x / w - 1.0) * (scene.camera_fov_x * 0.5).tan();
    let py = -(2.0 * real_y / h - 1.0) * (scene.camera_fov_y * 0.5).tan();
    let pz = 1.0f32;
    let direction = px * scene.camera_right + py * scene.camera_up + pz * scene.camera_forward;
    Ray {
        origin: scene.camera_position,
        direction,
    }
}

fn get_ray_color(ray: Ray, scene: &Scene) -> Vec3f {
    match intersect_ray_with_scene(ray, scene) {
        Some(intersection) => {
            intersection.0.color
        },
        None => scene.bg_color,
    }
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
                .map(|t| (primitive.clone(), t.clone()))
                .collect::<Vec<(Primitive, Intersection)>>()
        })
        .filter(|(_, intersection)| intersection.offset >= 0.0)
        .min_by(|a, b| a.1.offset.partial_cmp(&b.1.offset).unwrap())
}
