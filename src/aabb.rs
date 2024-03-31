use crate::geometry::{Object3D, Shape3D, Vec3f, EPS, FP_INF, FP_NEG_INF};
use crate::scene::Primitive;

#[derive(Clone, Debug, Default)]
pub struct AABB {
    pub min: Vec3f,
    pub max: Vec3f,
}

fn calculate_aabb_for_shape(shape3d: &Shape3D) -> AABB {
    let eps_vec = Vec3f::new(EPS, EPS, EPS);
    match shape3d {
        Shape3D::Plane { norm: _norm } => {
            panic!("AABB on plane")
        }
        Shape3D::Ellipsoid { r } => AABB {
            min: -r - eps_vec,
            max: r + eps_vec,
        },
        Shape3D::Box { s } => AABB {
            min: -s - eps_vec,
            max: s + eps_vec,
        },
        Shape3D::Triangle { a, b, c } => AABB {
            min: a.inf(b).inf(c) - eps_vec,
            max: a.sup(b).sup(c) + eps_vec,
        },
    }
}

// to make the formatter happy
fn if_then_else<T>(cond: bool, fst: T, snd: T) -> T {
    if cond {
        fst
    } else {
        snd
    }
}
pub fn calculate_aabb_for_object(object: &Object3D) -> AABB {
    let mut min = Vec3f::new(FP_INF, FP_INF, FP_INF);
    let mut max = Vec3f::new(FP_NEG_INF, FP_NEG_INF, FP_NEG_INF);
    let shape_aabb = calculate_aabb_for_shape(&object.shape);
    for x_from_min in [false, true] {
        for y_from_min in [false, true] {
            for z_from_min in [false, true] {
                let point = Vec3f::new(
                    if_then_else(x_from_min, shape_aabb.min.x, shape_aabb.max.x),
                    if_then_else(y_from_min, shape_aabb.min.y, shape_aabb.max.y),
                    if_then_else(z_from_min, shape_aabb.min.z, shape_aabb.max.z),
                );
                let object_point = object.rotation.transform_vector(&point) + object.position;
                min = min.inf(&object_point);
                max = max.sup(&object_point);
            }
        }
    }
    AABB { min, max }
}

pub fn calculate_aabb(slice: &[Primitive]) -> AABB {
    let mut min = Vec3f::new(FP_INF, FP_INF, FP_INF);
    let mut max = Vec3f::new(FP_NEG_INF, FP_NEG_INF, FP_NEG_INF);
    for a in slice {
        match a.object3d.shape {
            Shape3D::Plane { .. } => {
                continue;
            }
            _ => {
                let aabb = calculate_aabb_for_object(&a.object3d);
                min = min.inf(&aabb.min);
                max = max.sup(&aabb.max);
            }
        }
    }
    AABB { min, max }
}
