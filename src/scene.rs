use na::{Quaternion, UnitQuaternion, Vector3};

use crate::scene;

pub type Vec3f = Vector3<f64>;

#[derive(Clone, Debug)]
pub enum Shape3D {
    None,
    Plane { norm: Vec3f },
    Ellipsoid { rx: f64, ry: f64, rz: f64 },
    Box { sx: f64, sy: f64, sz: f64 },
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
}

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

#[derive(Debug)]
pub struct Scene {
    pub width: i32,
    pub height: i32,
    pub bg_color: Vec3f,
    pub camera_position: Vec3f,
    pub camera_forward: Vec3f,
    pub camera_right: Vec3f,
    pub camera_up: Vec3f,
    pub camera_fov_x: f64,
    pub camera_fov_y: f64,
    pub primitives: Vec<Primitive>,
    pub lights: Vec<LightSource>,
    pub ray_depth: i32,
    pub ambient_light: Vec3f,
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
        primitives: vec![],
        lights: vec![],
        ray_depth: 0,
        ambient_light: Default::default(),
    };

    let mut current_primitive = Primitive {
        shape: Shape3D::None,
        color: Default::default(),
        position: Default::default(),
        rotation: Default::default(),
        material: Material::Diffused,
        ior: 0.0,
    };

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
                if let Shape3D::None = current_primitive.shape {
                } else {
                    result.primitives.push(current_primitive.clone())
                }
                current_primitive = Primitive {
                    shape: Shape3D::None,
                    color: Default::default(),
                    position: Default::default(),
                    rotation: Default::default(),
                    material: Material::Diffused,
                    ior: 0.0,
                }
            }
            "PLANE" => current_primitive.shape = Shape3D::Plane { norm: get_vector() },
            "ELLIPSOID" => {
                current_primitive.shape = Shape3D::Ellipsoid {
                    rx: get_vector().x,
                    ry: get_vector().y,
                    rz: get_vector().z,
                }
            }
            "BOX" => {
                current_primitive.shape = Shape3D::Box {
                    sx: get_vector().x,
                    sy: get_vector().y,
                    sz: get_vector().z,
                }
            }
            "POSITION" => {
                current_primitive.position = get_vector();
            }
            "COLOR" => {
                current_primitive.color = get_vector();
            }
            "ROTATION" => {
                current_primitive.rotation = UnitQuaternion::new_normalize(Quaternion::new(
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
                current_primitive.material = Material::Metallic;
            }
            "DIELECTRIC" => {
                current_primitive.material = Material::Dielectric;
            }
            "RAY_DEPTH" => {
                result.ray_depth = tokens[1].parse().unwrap();
            },
            "IOR" => {
                current_primitive.ior = tokens[1].parse().unwrap();
            },
            "AMBIENT_LIGHT" => {
                result.ambient_light = get_vector();
            }
            _ => {
                // ignore unknown command
                // panic!("Unexpected command: {}", tokens[0])
            }
        }
    }
    result.primitives.push(current_primitive.clone());
    if !first_light_sourse {
        result.lights.push(current_light_source.clone());
    }
    result.camera_fov_y =
        ((result.camera_fov_x / 2.).tan() * result.height as f64 / result.width as f64).atan()
            * 2.0;

    println!("{:?}", result);
    result
}
