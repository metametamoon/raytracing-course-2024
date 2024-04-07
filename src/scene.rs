use na::{Quaternion, UnitQuaternion, Vector3};

use crate::aabb::{calculate_aabb_for_object, Aabb};
use crate::bvh::{create_bvh_tree, BvhTree};
use crate::geometry::{Fp, Material, Object3D, Shape3D, Vec3f, EPS};

#[derive(Clone, Debug)]
pub enum LightLocation {
    Directed { direction: Vec3f },
    Point { position: Vec3f, attenuation: Vec3f },
}

#[derive(Clone, Debug)]
pub struct LightSource {
    pub light_intensity: Vec3f,
    pub location: LightLocation,
}

#[derive(Clone, Debug)]
pub struct Primitive {
    pub object3d: Object3D,
    pub aabb: Aabb,
    pub color: Vec3f,
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
    pub lights: Vec<LightSource>,
    pub ray_depth: i32,
    pub ambient_light: Vec3f,
    pub samples: i32,
    pub infinite_primitives: Vec<Primitive>,
    pub bvh_light_sources: BvhTree,
}

pub fn parse_file_content(content: Vec<&str>) -> Scene {
    let mut result = Scene {
        width: 0,
        height: 0,
        bg_color: Default::default(),
        camera_position: Default::default(),
        camera_forward: Default::default(),
        camera_right: Default::default(),
        camera_up: Default::default(),
        camera_fov_x: 0.0,
        camera_fov_y: 0.0,
        bvh_finite_primitives: Default::default(),
        lights: vec![],
        ray_depth: 0,
        ambient_light: Default::default(),
        samples: 1,
        infinite_primitives: vec![],
        bvh_light_sources: Default::default(),
    };
    let mut finite_primitives = vec![];
    let mut light_emitting_finite_primitives = vec![];

    let mut current_primitive: Option<Primitive> = None;

    let mut current_light_source = LightSource {
        light_intensity: Vec3f::default(),
        location: LightLocation::Directed {
            direction: Vec3f::default(),
        },
    };
    let mut first_light_sourse = true;

    for line in content {
        let tokens: Vec<String> = line.split(' ').map(|x| x.to_string()).collect();

        let get_vector = || -> Vec3f {
            Vector3::new(
                tokens[1].parse().unwrap(),
                tokens[2].parse().unwrap(),
                tokens[3].parse().unwrap(),
            )
        };

        match tokens[0].as_str() {
            "DIMENSIONS" => {
                result.width = tokens[1].parse().unwrap();
                result.height = tokens[2].parse().unwrap();
            }
            "BG_COLOR" => {
                result.bg_color = get_vector();
            }
            "CAMERA_POSITION" => {
                result.camera_position = get_vector();
            }
            "CAMERA_FORWARD" => {
                result.camera_forward = get_vector();
            }
            "CAMERA_RIGHT" => {
                result.camera_right = get_vector();
            }
            "CAMERA_UP" => {
                result.camera_up = get_vector();
            }
            "CAMERA_FOV_X" => result.camera_fov_x = tokens[1].parse().unwrap(),
            "NEW_PRIMITIVE" => {
                if let Some(current_primitive) = current_primitive {
                    let aabb = match current_primitive.object3d.shape {
                        Shape3D::Plane { .. } => Default::default(),
                        _ => calculate_aabb_for_object(&current_primitive.object3d),
                    };
                    let primitive_to_push = Primitive {
                        object3d: current_primitive.object3d.clone(),
                        aabb,
                        color: current_primitive.color,
                        material: current_primitive.material,
                        ior: current_primitive.ior,
                        emission: current_primitive.emission,
                    };
                    match current_primitive.object3d.shape {
                        Shape3D::Plane { .. } => result.infinite_primitives.push(primitive_to_push),
                        _ => {
                            finite_primitives.push(primitive_to_push.clone());
                            if current_primitive.emission.norm() > EPS {
                                light_emitting_finite_primitives.push(primitive_to_push)
                            }
                        }
                    }
                }
                current_primitive = Some(Primitive {
                    object3d: Object3D {
                        shape: Shape3D::Box {
                            s: Default::default(),
                        },
                        position: Default::default(),
                        rotation: Default::default(),
                    },
                    aabb: Default::default(),
                    color: Default::default(),
                    material: Material::Diffused,
                    ior: 0.0,
                    emission: Default::default(),
                })
            }
            "PLANE" => {
                current_primitive.as_mut().unwrap().object3d.shape =
                    Shape3D::Plane { norm: get_vector() }
            }
            "ELLIPSOID" => {
                current_primitive.as_mut().unwrap().object3d.shape =
                    Shape3D::Ellipsoid { r: get_vector() }
            }
            "BOX" => {
                current_primitive.as_mut().unwrap().object3d.shape =
                    Shape3D::Box { s: get_vector() }
            }
            "TRIANGLE" => {
                current_primitive.as_mut().unwrap().object3d.shape = Shape3D::Triangle {
                    a: get_vector(),
                    b: Vec3f::new(
                        tokens[4].parse().unwrap(),
                        tokens[5].parse().unwrap(),
                        tokens[6].parse().unwrap(),
                    ),
                    c: Vec3f::new(
                        tokens[7].parse().unwrap(),
                        tokens[8].parse().unwrap(),
                        tokens[9].parse().unwrap(),
                    ),
                }
            }
            "POSITION" => {
                current_primitive.as_mut().unwrap().object3d.position = get_vector();
            }
            "COLOR" => {
                current_primitive.as_mut().unwrap().color = get_vector();
            }
            "ROTATION" => {
                current_primitive.as_mut().unwrap().object3d.rotation =
                    UnitQuaternion::new_normalize(Quaternion::new(
                        tokens[4].parse().unwrap(),
                        tokens[1].parse().unwrap(),
                        tokens[2].parse().unwrap(),
                        tokens[3].parse().unwrap(),
                    ))
            }
            "NEW_LIGHT" => {
                if !first_light_sourse {
                    result.lights.push(current_light_source.clone());
                }
                first_light_sourse = false;
                current_light_source = LightSource {
                    light_intensity: Vec3f::default(),
                    location: LightLocation::Directed {
                        direction: Vec3f::default(),
                    },
                };
            }
            "LIGHT_DIRECTION" => {
                current_light_source.location = LightLocation::Directed {
                    direction: get_vector(),
                };
            }
            "LIGHT_POSITION" => match current_light_source.location {
                LightLocation::Directed { direction: _ } => {
                    current_light_source.location = LightLocation::Point {
                        position: get_vector(),
                        attenuation: Default::default(),
                    }
                }
                LightLocation::Point {
                    position: _,
                    attenuation,
                } => {
                    current_light_source.location = LightLocation::Point {
                        position: get_vector(),
                        attenuation,
                    }
                }
            },
            "LIGHT_ATTENUATION" => match current_light_source.location {
                LightLocation::Directed { direction: _ } => {
                    current_light_source.location = LightLocation::Point {
                        position: Default::default(),
                        attenuation: get_vector(),
                    }
                }
                LightLocation::Point {
                    position,
                    attenuation: _,
                } => {
                    current_light_source.location = LightLocation::Point {
                        position,
                        attenuation: get_vector(),
                    }
                }
            },
            "LIGHT_INTENSITY" => {
                current_light_source.light_intensity = get_vector();
            }
            "METALLIC" => {
                current_primitive.as_mut().unwrap().material = Material::Metallic;
            }
            "DIELECTRIC" => {
                current_primitive.as_mut().unwrap().material = Material::Dielectric;
            }
            "RAY_DEPTH" => {
                result.ray_depth = tokens[1].parse().unwrap();
            }
            "IOR" => {
                current_primitive.as_mut().unwrap().ior = tokens[1].parse().unwrap();
            }
            "AMBIENT_LIGHT" => {
                result.ambient_light = get_vector();
            }
            "SAMPLES" => {
                result.samples = tokens[1].parse().unwrap();
            }
            "EMISSION" => {
                current_primitive.as_mut().unwrap().emission = get_vector();
            }
            _ => {
                // ignore unknown command
                // panic!("Unexpected command: {}", tokens[0])
            }
        }
    }
    if let Some(current_primitive) = current_primitive {
        let aabb = match current_primitive.object3d.shape {
            Shape3D::Plane { .. } => Default::default(),
            _ => calculate_aabb_for_object(&current_primitive.object3d),
        };
        let primitive_to_push = Primitive {
            object3d: current_primitive.object3d.clone(),
            aabb,
            color: current_primitive.color,
            material: current_primitive.material,
            ior: current_primitive.ior,
            emission: current_primitive.emission,
        };
        match current_primitive.object3d.shape {
            Shape3D::Plane { .. } => result.infinite_primitives.push(primitive_to_push),
            _ => {
                finite_primitives.push(primitive_to_push.clone());
                if current_primitive.emission.norm() > EPS {
                    light_emitting_finite_primitives.push(primitive_to_push)
                }
            }
        }
    }
    if !first_light_sourse {
        result.lights.push(current_light_source.clone());
    }
    result.camera_fov_y =
        ((result.camera_fov_x / 2.).tan() * result.height as Fp / result.width as Fp).atan() * 2.0;
    result.bvh_finite_primitives = create_bvh_tree(finite_primitives);
    result.bvh_light_sources = create_bvh_tree(light_emitting_finite_primitives);
    result
}
