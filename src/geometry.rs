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
pub enum Material {
    Dielectric,
    Metallic,
    Diffused,
}

#[derive(Clone, Debug)]
pub struct Primitive {
    pub shape: Shape3D,
    pub color: Vec3f,
    pub position: Vec3f,
    pub rotation: UnitQuaternion<f64>,
    pub material: Material,
    pub ior: f64,
    pub emission: Vec3f,
}

pub static EPS: f64 = 0.0001;

fn intersect(ray: Ray, shape: &Shape3D, upper_bound: f64) -> Option<Intersection> {
    match shape {
        Shape3D::None => {
            panic!("Intersect with None")
        }
        Shape3D::Plane { norm } => {
            let x = norm.dot(&ray.direction);
            if x == 0.0 {
                None
            } else {
                let offset = -ray.origin.dot(norm) / x;
                if 0.0 < offset && offset < upper_bound {
                    Some(Intersection {
                        offset,
                        normal: if x < 0.0 {
                            norm.normalize()
                        } else {
                            -norm.normalize()
                        },
                        is_outer_to_inner: true,
                    })
                } else {
                    None
                }
            }
        }
        Shape3D::Ellipsoid { r } => {
            let d1 = ray.direction.component_div(&r);
            let o1 = ray.origin.component_div(&r);
            let a = d1.dot(&d1);
            let b = 2.0 * o1.dot(&d1);
            let c = o1.dot(&o1) - 1.0;
            let discr = b * b - 4.0 * a * c;
            if discr < 0.0 {
                None
            } else {
                let x1 = (-b - discr.sqrt()) / (2.0 * a);
                let x2 = (-b + discr.sqrt()) / (2.0 * a);

                let t1 = f64::min(x1, x2);
                let t2 = f64::max(x1, x2);
                if t1 > 0.0 && t1 < upper_bound {
                    let p1 = ray.origin + ray.direction * t1;
                    let norm1 = p1.component_div(&r).component_div(&r).normalize();
                    return Some(Intersection {
                        offset: t1,
                        normal: norm1,
                        is_outer_to_inner: true,
                    });
                } else if t2 > 0.0 && t2 < upper_bound {
                    let p2 = ray.origin + ray.direction * t2;

                    let norm2 = p2.component_div(&r).component_div(&r).normalize();
                    Some(Intersection {
                        offset: t2,
                        normal: -norm2,
                        is_outer_to_inner: false,
                    })
                } else {
                    None
                }
            }
        }
        Shape3D::Box { s } => {
            let sx = s.x;
            let sy = s.y;
            let sz = s.z;
            let mut t_x = if ray.direction.x.abs() > EPS {
                [
                    (-sx - ray.origin.x) / ray.direction.x,
                    (sx - ray.origin.x) / ray.direction.x,
                ]
            } else {
                [-f64::INFINITY, f64::INFINITY]
            };
            t_x.sort_by(|a, b| a.partial_cmp(b).unwrap());

            let mut t_y = if ray.direction.y.abs() > EPS {
                [
                    (-sy - ray.origin.y) / ray.direction.y,
                    (sy - ray.origin.y) / ray.direction.y,
                ]
            } else {
                [-f64::INFINITY, f64::INFINITY]
            };
            t_y.sort_by(|a, b| a.partial_cmp(b).unwrap());

            let mut t_z = if ray.direction.z.abs() > EPS {
                [
                    (-sz - ray.origin.z) / ray.direction.z,
                    (sz - ray.origin.z) / ray.direction.z,
                ]
            } else {
                [-f64::INFINITY, f64::INFINITY]
            };
            t_z.sort_by(|a, b| a.partial_cmp(b).unwrap());

            let t_min = f64::max(t_x[0], f64::max(t_y[0], t_z[0]));
            let t_max = f64::min(t_x[1], f64::min(t_y[1], t_z[1]));
            if t_min < t_max {
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
                    Some(Intersection {
                        offset: t_min,
                        normal: norm_min,
                        is_outer_to_inner: true,
                    })
                } else if t_max > 0.0 && t_max < upper_bound {
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
                    Some(Intersection {
                        offset: t_max,
                        normal: -norm_max.normalize(),
                        is_outer_to_inner: false,
                    })
                } else {
                    None
                }
            } else {
                None
            }
        }
    }
}

pub fn get_reflection_ray(ray: &Vec3f, normal: &Vec3f) -> Vec3f {
    let projection = -ray.dot(normal);
    ray + normal * projection * 2.0
}

pub fn intersect_ray_with_primitive(
    ray: &Ray,
    primitive: &Primitive,
    min_dist: f64,
) -> Option<Intersection> {
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
    let x = rotated_ray.origin + rotated_ray.direction;
    intersect(rotated_ray, &primitive.shape, min_dist)
}