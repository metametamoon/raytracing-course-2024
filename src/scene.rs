use na::{Quaternion, UnitQuaternion, Vector3};

pub type Vec3f = Vector3<f32>;

#[derive(Clone, Debug)]
pub enum Shape3D {
    None,
    Plane { norm: Vec3f },
    Ellipsoid { rx: f32, ry: f32, rz: f32 },
    Box { sx: f32, sy: f32, sz: f32 },
}

#[derive(Clone, Debug)]
pub struct Primitive {
    pub shape: Shape3D,
    pub color: Vec3f,
    pub position: Vec3f,
    pub rotation: UnitQuaternion<f32>,
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
    pub camera_fov_x: f32,
    pub camera_fov_y: f32,
    pub primitives: Vec<Primitive>,
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
    };

    let mut current_primitive = Primitive {
        shape: Shape3D::None,
        color: Default::default(),
        position: Default::default(),
        rotation: Default::default(),
    };
    for line in content {
        let tokens: Vec<String> = line.split(' ').map(|x| x.to_string()).collect();

        let get_vector = || -> Vec3f {
            return Vector3::new(
                tokens[1].parse().unwrap(),
                tokens[2].parse().unwrap(),
                tokens[3].parse().unwrap(),
            );
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
            _ => {
                panic!("Unexpected command: {}", tokens[0])
            }
        }
    }
    result.primitives.push(current_primitive.clone());
    result.camera_fov_y =
        ((result.camera_fov_x / 2.).tan() * result.height as f32 / result.width as f32).atan()
            * 2.0;
    result
}
