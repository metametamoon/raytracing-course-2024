use arrayvec::ArrayVec;

use crate::aabb::{calculate_aabb, calculate_aabb_for_object, AABB};
use crate::geometry::{
    intersect_ray_with_object3d, intersect_ray_with_object3d_all_points, Fp, Intersection,
    Object3D, Ray, Shape3D, FP_INF,
};
use crate::scene::Primitive;

#[derive(Debug, Clone)]
struct BvhNode {
    aabb: AABB,
    left_child_index: usize,
    right_child_index: usize,
    content_start: usize,
    content_length: usize,
}

// owns its primitives
#[derive(Debug, Default, Clone)]
pub struct BvhTree {
    pub primitives: Vec<Primitive>,
    nodes: Vec<BvhNode>,
}

pub fn create_bvh_tree(finite_primitives: Vec<Primitive>) -> BvhTree {
    let mut finite_primitives = finite_primitives;
    let mut nodes = Vec::<BvhNode>::new();
    let len = finite_primitives.len();
    let last_index = create_bvh_node(&mut nodes, &mut finite_primitives, 0usize, len);
    assert_eq!(
        last_index + 1,
        nodes.len(),
        "Failed contract that root is at the end of array of nodes"
    );
    BvhTree {
        primitives: finite_primitives,
        nodes,
    }
}

// today, the battle with borrow checker was lost in trying to
// store a primitive slice as an actual slice, `&'a mut [Primitive]`. Too bad,
// this argument type does not allow to store the immutable slice because the mutable reference
// must be a unique owner and it does not die until the end of function. But we will meet in battle
// again!
fn create_bvh_node(
    result: &mut Vec<BvhNode>,
    primitives: &mut Vec<Primitive>,
    start: usize,
    length: usize,
) -> usize {
    println!("Called bvh! len={}", length);
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
    let n = primitives.len();
    if n <= 4 {
        return None;
    }
    let mut current_best_cost = FP_INF;
    let mut current_best_coord = 0;
    let mut current_best_index = usize::MAX;

    for coord in 0..3 {
        primitives.sort_by(|x, y| {
            let aabb_x = calculate_aabb_for_object(&x.object3d);
            let aabb_y = calculate_aabb_for_object(&y.object3d);
            let center_x = aabb_x.min + aabb_x.max;
            let center_y = aabb_y.min + aabb_y.max;
            center_x[(coord, 0)]
                .partial_cmp(&center_y[(coord, 0)])
                .unwrap()
        });
        let mut left_prefixes = vec![];
        let mut right_prefixes = vec![];
        let mut current_left_prefix = <AABB as Default>::default();
        let mut current_right_prefix = <AABB as Default>::default();
        for i in 0..(n - 1) {
            current_left_prefix = current_left_prefix.extend_aabb(&primitives[i].aabb);
            current_right_prefix = current_right_prefix.extend_aabb(&primitives[n - 1 - i].aabb);
            left_prefixes.push(current_left_prefix.area());
            right_prefixes.push(current_right_prefix.area());
        }
        for i in 0..(n - 1) {
            let left_cost = ((i + 1) as Fp) * left_prefixes[i];
            let right_cost = ((n - (i + 1)) as Fp) * right_prefixes[n - (i + 1) - 1];
            let current_cost = left_cost + right_cost;
            if current_cost < current_best_cost {
                current_best_cost = current_cost;
                current_best_coord = coord;
                current_best_index = i;
            }
        }
    }
    let trivial_cost = aabb.area() * (n as Fp);
    if trivial_cost < current_best_cost {
        return None;
    }
    primitives.sort_by(|x, y| {
        let aabb_x = calculate_aabb_for_object(&x.object3d);
        let aabb_y = calculate_aabb_for_object(&y.object3d);
        let center_x = aabb_x.min + aabb_x.max;
        let center_y = aabb_y.min + aabb_y.max;
        center_x[(current_best_coord, 0)].total_cmp(&center_y[(current_best_coord, 0)])
    });
    let count = current_best_index + 1;
    if count >= 1 && (n - count) >= 1 {
        Some(count)
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

pub struct BvhIntersection<'a> {
    pub intersections: ArrayVec<Intersection, 2>,
    pub rotated_ray: Ray,
    pub primitive: &'a Primitive,
}

pub fn intersect_with_bvh_all_points<'a>(
    ray: &Ray,
    bvh_tree: &'a BvhTree,
) -> Vec<BvhIntersection<'a>> {
    let root_index = bvh_tree.nodes.len() - 1;
    let mut result = vec![];
    intersect_with_bvh_all_points_impl(
        ray,
        &bvh_tree.primitives,
        &bvh_tree.nodes,
        root_index,
        &mut result,
    );
    result
}

fn intersect_with_bvh_all_points_impl<'a>(
    ray: &Ray,
    primitives: &'a Vec<Primitive>,
    bvh_nodes: &Vec<BvhNode>,
    bvh_node_index: usize,
    result: &mut Vec<BvhIntersection<'a>>,
) {
    let current_node = &bvh_nodes[bvh_node_index];
    if intersects(ray, &current_node.aabb) {
        if current_node.left_child_index == usize::MAX {
            let node_primitives = &primitives[current_node.content_start
                ..current_node.content_start + current_node.content_length];
            for primitive in node_primitives {
                let points = intersect_ray_with_object3d_all_points(ray, &primitive.object3d);
                result.push(BvhIntersection {
                    intersections: points.0,
                    rotated_ray: points.1,
                    primitive,
                });
            }
        } else {
            intersect_with_bvh_all_points_impl(
                ray,
                primitives,
                bvh_nodes,
                current_node.left_child_index,
                result,
            );
            intersect_with_bvh_all_points_impl(
                ray,
                primitives,
                bvh_nodes,
                current_node.right_child_index,
                result,
            );
        }
    }
}

pub fn validate_bvh(bvh_tree: &BvhTree) {
    for bvh_node in &bvh_tree.nodes {
        if bvh_node.left_child_index == usize::MAX {
            let primitives = &bvh_tree.primitives
                [bvh_node.content_start..bvh_node.content_start + bvh_node.content_length];
            for primitive in primitives {
                let primitive_aabb = calculate_aabb_for_object(&primitive.object3d);
                assert!(
                    bvh_node.aabb.min.x <= primitive_aabb.min.x
                        && bvh_node.aabb.min.y <= primitive_aabb.min.y
                        && bvh_node.aabb.min.z <= primitive_aabb.min.z
                        && bvh_node.aabb.max.x >= primitive_aabb.max.x
                        && bvh_node.aabb.max.y >= primitive_aabb.max.y
                        && bvh_node.aabb.max.z >= primitive_aabb.max.z
                );
            }
        } else {
            // let left_child = &bvh_tree.nodes[bvh_node.left_child_index];
            // let right_child = &bvh_tree.nodes[bvh_node.right_child_index];
        }
    }
}
