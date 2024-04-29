use crate::aabb::Aabb;
use crate::bvh::BvhTree;
use crate::geometry::{Fp, Object3D, Vec3f};


#[derive(Clone, Debug)]
pub struct Material {
    pub base_color_factor: Vec3f,
    pub metallic_factor: Fp,
    pub metallic_roughness: Fp,
}

#[derive(Clone, Debug)]
pub struct Primitive {
    pub object3d: Object3D,
    pub aabb: Aabb,
    pub material: Material,
    pub ior: Fp,
    pub emission: Vec3f,
}

#[derive(Debug)]
pub struct Scene {
    pub width: i32,
    pub height: i32,
    pub bg_color: Vec3f,
    pub camera_position: Vec3f,
    pub camera_forward: Vec3f,
    pub camera_right: Vec3f,
    pub camera_up: Vec3f,
    pub camera_fov_x: Fp,
    pub camera_fov_y: Fp,
    pub bvh_finite_primitives: BvhTree,
    pub ray_depth: i32,
    pub ambient_light: Vec3f,
    pub samples: i32,
    pub infinite_primitives: Vec<Primitive>,
    pub bvh_light_sources: BvhTree,
}
