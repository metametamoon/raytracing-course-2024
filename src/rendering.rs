use crate::scene::Scene;
use crate::scene::Shape3D;
use crate::scene::Vec3f;
use na::Vector3;

struct Ray {
    origin: Vec3f,
    direction: Vec3f,
}

fn intersect(ray: Ray, shape: &Shape3D) -> Vec<f32> {
    match shape {
        Shape3D::None => {
            panic!("Intersect with None")
        }
        Shape3D::Plane { norm } => {
            let x = norm.dot(&ray.direction);
            if x == 0.0 {
                vec![]
            } else {
                vec![-ray.origin.dot(&norm) / x]
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
            if discr >= 0.0 {
                vec![
                    (-b - discr.sqrt()) / (2.0 * a),
                    (-b + discr.sqrt()) / (2.0 * a),
                ]
            } else {
                vec![]
            }
        }
        Shape3D::Box { sx, sy, sz } => {
            if ray.direction.x == 0.0 || ray.direction.y == 0.0 || ray.direction.z == 0.0 {
                panic!("I pray this is unreachable")
            } else {
                let mut t_x = vec![
                    (-sx - ray.origin.x) / ray.direction.x,
                    (sx - ray.origin.x) / ray.direction.x,
                ];
                t_x.sort_by(|a, b| a.partial_cmp(b).unwrap());

                let mut t_y = vec![
                    (-sy - ray.origin.y) / ray.direction.y,
                    (sy - ray.origin.y) / ray.direction.y,
                ];
                t_y.sort_by(|a, b| a.partial_cmp(b).unwrap());

                let mut t_z = vec![
                    (-sz - ray.origin.z) / ray.direction.z,
                    (sz - ray.origin.z) / ray.direction.z,
                ];
                t_z.sort_by(|a, b| a.partial_cmp(b).unwrap());

                let t_min = f32::max(t_x[0], f32::max(t_y[0], t_z[0]));
                let t_max = f32::min(t_x[1], f32::min(t_y[1], t_z[1]));
                if t_min < t_max {
                    vec![t_min, t_max]
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
            let real_x = x as f32 + 0.5;
            let real_y = y as f32 + 0.5;
            let w = scene.width as f32;
            let h = scene.height as f32;
            let px = (2.0 * real_x / w - 1.0) * (scene.camera_fov_x * 0.5).tan();
            let py = -(2.0 * real_y / h - 1.0) * (scene.camera_fov_y * 0.5).tan();
            let pz = 1.0f32;
            let direction = px * scene.camera_right + py * scene.camera_up + pz * scene.camera_forward;
            let ray_to_pixel: Ray = Ray { origin: scene.camera_position, direction: direction };
            let first_visible = scene
                .primitives
                .iter()
                .flat_map(|primitive| {
                    let transposed_ray = Ray {
                        origin: ray_to_pixel.origin - primitive.position,
                        direction: ray_to_pixel.direction,
                    };
                    let rotated_ray = Ray {
                        origin: primitive.rotation.conjugate().transform_vector(&transposed_ray.origin),
                        direction: primitive
                            .rotation
                            .conjugate()
                            .transform_vector(&transposed_ray.direction),
                    };

                    intersect(rotated_ray, &primitive.shape)
                        .iter()
                        .map(|t| (*t, primitive.color.clone()))
                        .collect::<Vec<(f32, Vec3f)>>()
                })
                .filter(|(t, _)| *t >= 0.0)
                .min_by(|a, b| a.0.partial_cmp(&b.0).unwrap());
            let color = first_visible.map(|x| x.1).unwrap_or(scene.bg_color);
            result.push((color.x * 255.0).round() as u8);
            result.push((color.y * 255.0).round() as u8);
            result.push((color.z * 255.0).round() as u8);
        }
    }
    result
}
