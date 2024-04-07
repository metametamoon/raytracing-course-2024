use arrayvec::ArrayVec;
use na::Matrix3;
use nalgebra::{UnitQuaternion, Vector3};

pub type Fp = f32;
pub const FP_PI: Fp = std::f32::consts::PI;
pub const FP_INF: Fp = f32::INFINITY;
pub const FP_NEG_INF: Fp = f32::NEG_INFINITY;
pub type Vec3f = Vector3<Fp>;

#[derive(Clone, Debug)]
pub struct Ray {
    pub origin: Vec3f,
    pub direction: Vec3f,
}

#[derive(Default, Clone)]
pub struct Intersection {
    pub offset: Fp,
    pub normal: Vec3f,
    pub is_outer_to_inner: bool,
}

#[derive(Clone, Debug)]
pub enum Shape3D {
    Plane { norm: Vec3f },
    Ellipsoid { r: Vec3f },
    Box { s: Vec3f },
    Triangle { a: Vec3f, b: Vec3f, c: Vec3f },
}

#[derive(Clone, Debug)]
pub struct Object3D {
    pub shape: Shape3D,
    pub position: Vec3f,
    pub rotation: UnitQuaternion<Fp>,
}

#[derive(Clone, Debug)]
pub enum Material {
    Dielectric,
    Metallic,
    Diffused,
}

pub static EPS: Fp = 0.00001;

fn intersect(ray: &Ray, shape: &Shape3D, upper_bound: Fp) -> Option<Intersection> {
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

fn sort2(x: Fp, y: Fp) -> (Fp, Fp) {
    if x < y {
        (x, y)
    } else {
        (y, x)
    }
}

fn intersect_all_points(ray: &Ray, shape: &Shape3D, upper_bound: Fp) -> ArrayVec<Intersection, 2> {
    match shape {
        Shape3D::Plane { norm } => intersect_with_plane(ray, upper_bound, norm),
        Shape3D::Ellipsoid { r } => intersect_with_ellipsoid(ray, upper_bound, r),
        Shape3D::Box { s } => intersect_with_box(ray, upper_bound, s),
        Shape3D::Triangle {
            a: p1,
            b: p2,
            c: p3,
        } => intersect_with_triangle(ray, upper_bound, p1, p2, p3),
    }
}

fn intersect_with_triangle(
    ray: &Ray,
    upper_bound: Fp,
    a: &Vec3f,
    b: &Vec3f,
    c: &Vec3f,
) -> ArrayVec<Intersection, 2> {
    let mut s = Matrix3::<Fp>::zeros();
    s.set_column(0, &(b - a));
    s.set_column(1, &(c - a));
    s.set_column(2, &-ray.direction);

    let mut result = ArrayVec::<Intersection, 2>::new();
    match s.try_inverse() {
        Some(s_inv) => {
            let uvt = s_inv * (ray.origin - a);
            let (u, v, t) = (uvt.x, uvt.y, uvt.z);
            if 0.0 <= u && 0.0 <= v && u + v <= 1.0 && 0.0 < t && t < upper_bound {
                let outer_normal = (b - a).cross(&(c - a)).normalize();
                let (is_outer_to_inner, normal) = if outer_normal.dot(&ray.direction) < 0.0 {
                    (true, outer_normal)
                } else {
                    (false, -outer_normal)
                };
                result.push(Intersection {
                    offset: t,
                    normal,
                    is_outer_to_inner,
                })
            }
            result
        }
        None => result,
    }
}

fn intersect_with_box(ray: &Ray, upper_bound: Fp, s: &Vec3f) -> ArrayVec<Intersection, 2> {
    let sx = s.x;
    let sy = s.y;
    let sz = s.z;
    let (t_x0, t_x1) = sort2(
        (-sx - ray.origin.x) / (ray.direction.x + 0.001 * EPS),
        (sx - ray.origin.x) / (ray.direction.x + 0.001 * EPS),
    );
    let (t_y0, t_y1) = sort2(
        (-sy - ray.origin.y) / (ray.direction.y + 0.001 * EPS),
        (sy - ray.origin.y) / (ray.direction.y + 0.001 * EPS),
    );
    let (t_z0, t_z1) = sort2(
        (-sz - ray.origin.z) / (ray.direction.z + 0.001 * EPS),
        (sz - ray.origin.z) / (ray.direction.z + 0.001 * EPS),
    );

    let t_min = Fp::max(t_x0, Fp::max(t_y0, t_z0));
    let t_max = Fp::min(t_x1, Fp::min(t_y1, t_z1));
    if t_min <= t_max {
        let mut result = ArrayVec::<Intersection, 2>::new();
        let calculate_norm = |p_on_box: Vec3f| -> Vec3f {
            if s.x - p_on_box.x.abs() < EPS {
                Vec3f::new(p_on_box.x.signum(), 0.0, 0.0)
            } else if s.y - p_on_box.y.abs() < EPS {
                Vec3f::new(0.0, p_on_box.y.signum(), 0.0)
            } else {
                Vec3f::new(0.0, 0.0, p_on_box.z.signum())
            }
        };
        if t_min > 0.0 && t_min < upper_bound {
            let p_min = ray.origin + ray.direction * t_min;
            let norm_min = calculate_norm(p_min);
            result.push(Intersection {
                offset: t_min,
                normal: norm_min,
                is_outer_to_inner: true,
            })
        }
        if t_max > 0.0 && t_max < upper_bound {
            let p_max = ray.origin + ray.direction * t_max;
            let norm_max = calculate_norm(p_max);
            result.push(Intersection {
                offset: t_max,
                normal: -norm_max,
                is_outer_to_inner: false,
            })
        }
        result
    } else {
        ArrayVec::<Intersection, 2>::new()
    }
}

fn intersect_with_ellipsoid(ray: &Ray, upper_bound: Fp, r: &Vec3f) -> ArrayVec<Intersection, 2> {
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
        let (t1, t2) = sort2(x1, x2);
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

fn intersect_with_plane(ray: &Ray, upper_bound: Fp, norm: &Vec3f) -> ArrayVec<Intersection, 2> {
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

pub fn intersect_ray_with_object3d(
    ray: &Ray,
    object: &Object3D,
    min_dist: Fp,
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
    if let Some(mut intersection) = intersect(&rotated_ray, &object.shape, min_dist) {
        intersection.normal = object.rotation.transform_vector(&intersection.normal);
        Some(intersection)
    } else {
        None
    }
}

// returns (intersections, local coord ray);
pub fn intersect_ray_with_object3d_all_points(
    ray: &Ray,
    object3d: &Object3D,
) -> (ArrayVec<Intersection, 2>, Ray) {
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
    let mut result = intersect_all_points(&rotated_ray, &object3d.shape, Fp::INFINITY);
    for intersection in &mut result {
        intersection.normal = object3d.rotation.transform_vector(&intersection.normal);
    }
    (result, rotated_ray)
}
