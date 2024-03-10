use arrayvec::ArrayVec;
use nalgebra::{UnitQuaternion, Vector3};

pub type Vec3f = Vector3<f64>;

#[derive(Clone, Debug)]
pub struct Ray {
    pub origin: Vec3f,
    pub direction: Vec3f,
}

#[derive(Default, Clone)]
pub struct Intersection {
    pub offset: f64,
    pub normal: Vec3f,
    pub is_outer_to_inner: bool,
}

#[derive(Clone, Debug)]
pub enum Shape3D {
    None,
    Plane { norm: Vec3f },
    Ellipsoid { r: Vec3f },
    Box { s: Vec3f },
}

#[derive(Clone, Debug)]
pub struct Object3D {
    pub shape: Shape3D,
    pub position: Vec3f,
    pub rotation: UnitQuaternion<f64>,
}

#[derive(Clone, Debug)]
pub enum Material {
    Dielectric,
    Metallic,
    Diffused,
}

pub static EPS: f64 = 0.00001;

fn intersect(ray: Ray, shape: &Shape3D, upper_bound: f64) -> Option<Intersection> {
    let mut intersections = intersect_all_points(ray, shape, upper_bound);
    if !intersections.is_empty() {
        Some(intersections.remove(0))
    } else {
        None
    }
}

pub fn get_reflection_ray(ray: &Vec3f, normal: &Vec3f) -> Vec3f {
    let projection = -ray.dot(normal);
    ray + normal * projection * 2.0
}

fn intersect_all_points(ray: Ray, shape: &Shape3D, upper_bound: f64) -> ArrayVec<Intersection, 2> {
    match shape {
        Shape3D::None => {
            panic!("Intersect with None")
        }
        Shape3D::Plane { norm } => {
            let x = norm.dot(&ray.direction);
            if x == 0.0 {
                ArrayVec::new()
            } else {
                let offset = -ray.origin.dot(norm) / x;
                let mut result = ArrayVec::<Intersection, 2>::new();
                if 0.0 < offset && offset < upper_bound {
                    result.push(Intersection {
                        offset,
                        normal: if x < 0.0 {
                            norm.normalize()
                        } else {
                            -norm.normalize()
                        },
                        is_outer_to_inner: true,
                    })
                }
                result
            }
        }
        Shape3D::Ellipsoid { r } => {
            let d1 = ray.direction.component_div(r);
            let o1 = ray.origin.component_div(r);
            let a = d1.dot(&d1);
            let b = 2.0 * o1.dot(&d1);
            let c = o1.dot(&o1) - 1.0;
            let discr = b * b - 4.0 * a * c;
            if discr < 0.0 {
                ArrayVec::new()
            } else {
                let x1 = (-b - discr.sqrt()) / (2.0 * a);
                let x2 = (-b + discr.sqrt()) / (2.0 * a);

                let t1 = f64::min(x1, x2);
                let t2 = f64::max(x1, x2);
                let mut result = ArrayVec::<Intersection, 2>::new();
                if t1 > 0.0 && t1 < upper_bound {
                    let p1 = ray.origin + ray.direction * t1;
                    let norm1 = p1.component_div(r).component_div(r).normalize();
                    result.push(Intersection {
                        offset: t1,
                        normal: norm1,
                        is_outer_to_inner: true,
                    });
                }
                if t2 > 0.0 && t2 < upper_bound {
                    let p2 = ray.origin + ray.direction * t2;

                    let norm2 = p2.component_div(r).component_div(r).normalize();
                    result.push(Intersection {
                        offset: t2,
                        normal: -norm2,
                        is_outer_to_inner: false,
                    });
                }
                result
            }
        }
        Shape3D::Box { s } => {
            let sx = s.x;
            let sy = s.y;
            let sz = s.z;
            let mut t_x = [
                (-sx - ray.origin.x) / (ray.direction.x + 0.001 * EPS),
                (sx - ray.origin.x) / (ray.direction.x + 0.001 * EPS),
            ];
            t_x.sort_by(|a, b| a.partial_cmp(b).unwrap());

            let mut t_y = [
                (-sy - ray.origin.y) / (ray.direction.y + 0.001 * EPS),
                (sy - ray.origin.y) / (ray.direction.y + 0.001 * EPS),
            ];
            t_y.sort_by(|a, b| a.partial_cmp(b).unwrap());

            let mut t_z = [
                (-sz - ray.origin.z) / (ray.direction.z + 0.001 * EPS),
                (sz - ray.origin.z) / (ray.direction.z + 0.001 * EPS),
            ];
            t_z.sort_by(|a, b| a.partial_cmp(b).unwrap());

            let t_min = f64::max(t_x[0], f64::max(t_y[0], t_z[0]));
            let t_max = f64::min(t_x[1], f64::min(t_y[1], t_z[1]));
            if t_min < t_max {
                let mut result = ArrayVec::<Intersection, 2>::new();
                if t_min > 0.0 && t_min < upper_bound {
                    let p_min = ray.origin + ray.direction * t_min;
                    let norm_min = {
                        if (p_min.x / s.x).abs() > 1.0 - EPS {
                            Vec3f::new((p_min.x / s.x).signum(), 0.0, 0.0)
                        } else if (p_min.y / s.y).abs() > 1.0 - EPS {
                            Vec3f::new(0.0, (p_min.y / s.y).signum(), 0.0)
                        } else {
                            Vec3f::new(0.0, 0.0, (p_min.z / s.z).signum())
                        }
                    };
                    result.push(Intersection {
                        offset: t_min,
                        normal: norm_min,
                        is_outer_to_inner: true,
                    })
                }
                if t_max > 0.0 && t_max < upper_bound {
                    let p_max = ray.origin + ray.direction * t_max;
                    let norm_max = {
                        if (p_max.x / s.x).abs() > 1.0 - EPS {
                            Vec3f::new((p_max.x / s.x).signum(), 0.0, 0.0)
                        } else if (p_max.y / s.y).abs() > 1.0 - EPS {
                            Vec3f::new(0.0, (p_max.y / s.y).signum(), 0.0)
                        } else {
                            Vec3f::new(0.0, 0.0, (p_max.z / s.z).signum())
                        }
                    };
                    result.push(Intersection {
                        offset: t_max,
                        normal: -norm_max.normalize(),
                        is_outer_to_inner: false,
                    })
                }
                result
            } else {
                ArrayVec::<Intersection, 2>::new()
            }
        }
    }
}

pub fn intersect_ray_with_object3d(
    ray: &Ray,
    object: &Object3D,
    min_dist: f64,
) -> Option<Intersection> {
    let transposed_ray = Ray {
        origin: ray.origin - object.position,
        direction: ray.direction,
    };
    let rotated_ray = Ray {
        origin: object
            .rotation
            .conjugate()
            .transform_vector(&transposed_ray.origin),
        direction: object
            .rotation
            .conjugate()
            .transform_vector(&transposed_ray.direction),
    };
    if let Some(mut intersection) = intersect(rotated_ray, &object.shape, min_dist) {
        intersection.normal = object.rotation.transform_vector(&intersection.normal);
        Some(intersection)
    } else {
        None
    }
}

pub fn intersect_ray_with_object3d_all_points(
    ray: &Ray,
    object3d: &Object3D,
) -> ArrayVec<Intersection, 2> {
    let transposed_ray = Ray {
        origin: ray.origin - object3d.position,
        direction: ray.direction,
    };
    let rotated_ray = Ray {
        origin: object3d
            .rotation
            .conjugate()
            .transform_vector(&transposed_ray.origin),
        direction: object3d
            .rotation
            .conjugate()
            .transform_vector(&transposed_ray.direction),
    };
    let mut result = intersect_all_points(rotated_ray, &object3d.shape, f64::INFINITY);
    for intersection in &mut result {
        intersection.normal = object3d.rotation.transform_vector(&intersection.normal);
    }
    result
}
