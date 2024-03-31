use crate::aabb::{calculate_aabb, AABB};
use crate::geometry::{
    intersect_ray_with_object3d, intersect_ray_with_object3d_all_points, Intersection, Object3D,
    Ray, Shape3D, FP_INF,
};
use crate::scene::Primitive;
use arrayvec::ArrayVec;

#[derive(Debug)]
pub struct BvhNode {
    aabb: AABB,
    left_child_index: usize,
    right_child_index: usize,
    content_start: usize,
    content_length: usize,
}

// today, the battle with borrow checker was lost in trying to
// store a primitive slice as an actual slice, `&'a mut [Primitive]`. Too bad,
// this argument type does not allow to store the immutable slice because the mutable reference
// must be a unique owner and it does not die until the end of function. But we will meet in battle
// again!
pub fn create_bvh_node(
    result: &mut Vec<BvhNode>,
    primitives: &mut Vec<Primitive>,
    start: usize,
    length: usize,
) -> usize {
    println!("Called bvh!");
    let aabb = calculate_aabb(&primitives[start..start + length]);
    match try_split(&aabb, &mut primitives[start..start + length]) {
        Some(first_part_len) => {
            let left_child = create_bvh_node(result, primitives, start, first_part_len);
            let right_child = create_bvh_node(
                result,
                primitives,
                start + first_part_len,
                length - first_part_len,
            );
            result.push(BvhNode {
                aabb,
                left_child_index: left_child,
                right_child_index: right_child,
                content_start: start,
                content_length: length,
            });
            result.len() - 1
        }
        None => {
            result.push(BvhNode {
                aabb,
                left_child_index: usize::MAX,
                right_child_index: usize::MAX,
                content_start: start,
                content_length: length,
            });
            result.len() - 1
        }
    }
}

// returns the length of the first part, if the split was successful
fn try_split(aabb: &AABB, primitives: &mut [Primitive]) -> Option<usize> {
    let diff = aabb.max - aabb.min;
    let x_len = diff.x;
    let y_len = diff.y;
    let z_len = diff.z;
    let first_part = if x_len >= y_len && x_len >= z_len {
        primitives
            .iter_mut()
            .partition_in_place(|p| p.aabb.min.x <= aabb.min.x + x_len * 0.5)
    } else if y_len >= x_len && y_len >= z_len {
        primitives
            .iter_mut()
            .partition_in_place(|p| p.aabb.min.y <= aabb.min.y + y_len * 0.5)
    } else {
        primitives
            .iter_mut()
            .partition_in_place(|p| p.aabb.min.z <= aabb.min.z + z_len * 0.5)
    };
    if 0usize < first_part && first_part < primitives.len() {
        Some(first_part)
    } else {
        None
    }
}

fn intersects(ray: &Ray, aabb: &AABB) -> bool {
    let object = Object3D {
        shape: Shape3D::Box {
            s: (aabb.max - aabb.min) * 0.5,
        },
        position: (aabb.max + aabb.min) * 0.5,
        rotation: Default::default(),
    };
    intersect_ray_with_object3d(ray, &object, FP_INF).is_some()
}

// ray in the return type is the rotated ray
pub fn intersect_with_bvh_all_points<'a>(
    ray: &Ray,
    primitives: &'a Vec<Primitive>,
    bvh_nodes: &Vec<BvhNode>,
    bvh_node_index: usize,
) -> Vec<(ArrayVec<Intersection, 2>, Ray, &'a Primitive)> {
    let current_node = &bvh_nodes[bvh_node_index];
    if intersects(ray, &current_node.aabb) {
        if current_node.left_child_index == usize::MAX {
            let mut result = vec![];
            let node_primitives = &primitives[current_node.content_start
                ..current_node.content_start + current_node.content_length];
            for primitive in node_primitives {
                let points = intersect_ray_with_object3d_all_points(ray, &primitive.object3d);
                result.push((points.0, points.1, primitive))
            }
            result
        } else {
            let mut result = intersect_with_bvh_all_points(
                ray,
                primitives,
                bvh_nodes,
                current_node.left_child_index,
            );
            result.extend(intersect_with_bvh_all_points(
                ray,
                primitives,
                bvh_nodes,
                current_node.right_child_index,
            ));
            result
        }
    } else {
        vec![]
    }
}
