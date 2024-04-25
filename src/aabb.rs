use crate::geometry::{Fp, Object3D, Shape3D, Vec3f, EPS, FP_INF, FP_NEG_INF};
use crate::scene::Primitive;

#[derive(Clone, Debug)]
pub struct Aabb {
    pub min: Vec3f,
    pub max: Vec3f,
}

impl Default for Aabb {
    fn default() -> Self {
        Aabb {
            min: Vec3f::new(FP_INF, FP_INF, FP_INF),
            max: Vec3f::new(FP_NEG_INF, FP_NEG_INF, FP_NEG_INF),
        }
    }
}
impl Aabb {
    pub fn _extend_point(&self, point: Vec3f) -> Aabb {
        Aabb {
            min: self.min.inf(&point),
            max: self.max.sup(&point),
        }
    }
    pub fn extend_aabb(&self, aabb: &Aabb) -> Aabb {
        Aabb {
            min: self.min.inf(&aabb.min),
            max: self.max.sup(&aabb.max),
        }
    }

    pub fn area(&self) -> Fp {
        let diff = self.max - self.min;
        let x = diff.x;
        let y = diff.y;
        let z = diff.z;
        x * y + y * z + z * x
    }

    pub fn contains(&self, aabb: &Aabb) -> bool {
        for coord in 0..3 {
            if aabb.min[(coord, 0)] < self.min[(coord, 0)] {
                return false;
            }
            if aabb.max[(coord, 0)] > self.max[(coord, 0)] {
                return false;
            }
        }
        true
    }
}

fn calculate_aabb_for_shape(shape3d: &Shape3D) -> Aabb {
    let eps_vec = Vec3f::new(EPS, EPS, EPS);
    match shape3d {
        Shape3D::Box { s } => Aabb {
            min: -s - eps_vec,
            max: s + eps_vec,
        },
        Shape3D::Triangle { a, b, c, .. } => Aabb {
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
pub fn calculate_aabb_for_object(object: &Object3D) -> Aabb {
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
    Aabb { min, max }
}

pub fn calculate_aabb(slice: &[Primitive]) -> Aabb {
    let mut result = <Aabb as Default>::default();
    for a in slice {
        match a.object3d.shape {
            _ => {
                result = result.extend_aabb(&calculate_aabb_for_object(&a.object3d));
            }
        }
    }
    result
}
