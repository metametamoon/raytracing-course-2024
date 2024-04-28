use gltf::buffer::Data;
use gltf::camera::Projection::Perspective;
use gltf::mesh::util::ReadIndices;
use gltf::Document;
use na::{Matrix4, Matrix4x1, Quaternion, UnitQuaternion};

use crate::aabb::calculate_aabb_for_object;
use crate::bvh::create_bvh_tree;
use crate::geometry::{Fp, Object3D, Shape3D, Vec3f, Vec4f, EPS};
use crate::scene::{Material, MaterialEnumerated, Primitive, Scene};

#[derive(Debug)]
struct Camera {
    pub camera_position: Vec3f,
    pub camera_forward: Vec3f,
    pub camera_right: Vec3f,
    pub camera_up: Vec3f,
    pub camera_fov_x: Fp,
    pub camera_fov_y: Fp,
}
pub fn convert_gltf_to_scene(
    gltf: &Document,
    buffers: &Vec<Data>,
    width: i32,
    height: i32,
    samples: i32,
) -> Scene {
    let default_transformation = Matrix4::<Fp>::identity();
    let default_rotation = Quaternion::<Fp>::identity();
    let mut camera = Camera {
        camera_position: Default::default(),
        camera_forward: Default::default(),
        camera_right: Default::default(),
        camera_up: Default::default(),
        camera_fov_x: 0.0,
        camera_fov_y: 0.0,
    };
    let mut finite_primitives = Vec::<Primitive>::new();
    let mut light_primitives = Vec::<Primitive>::new();
    let scenes = gltf.scenes().collect::<Vec<_>>();
    dbg!(scenes.len());
    for node in gltf.nodes() {
        read_primitives(
            &mut camera,
            &mut finite_primitives,
            &mut light_primitives,
            &buffers,
            &node,
            &default_transformation,
            &default_rotation,
        )
    }
    // for gltf_scene in scenes {
    //     for node in gltf_scene.nodes() {
    //         read_primitives(&mut camera, &mut finite_primitives, &mut light_primitives, &buffers, &node, &default_transformation);
    //     }
    // }
    // eprintln!("here!");
    dbg!(&camera);
    dbg!(finite_primitives.len());
    dbg!(light_primitives.len());
    Scene {
        width,
        height,
        bg_color: Vec3f::new(0.0, 0.0, 0.0),
        camera_position: camera.camera_position,
        camera_forward: camera.camera_forward,
        camera_right: camera.camera_right,
        camera_up: camera.camera_up,
        camera_fov_x: camera.camera_fov_x,
        camera_fov_y: camera.camera_fov_y,
        bvh_finite_primitives: create_bvh_tree(finite_primitives),
        ray_depth: 6,
        ambient_light: Default::default(),
        samples,
        infinite_primitives: vec![],
        bvh_light_sources: create_bvh_tree(light_primitives),
    }
}

fn pp4_to_r3(v: Vec4f) -> Vec3f {
    Vec3f::new(v.x / v.w, v.y / v.w, v.z / v.w)
}

fn pp4_vector_slice(v: Vec4f) -> Vec3f {
    Vec3f::new(v.x, v.y, v.z)
}

fn slice3_to_vec3f(p: &[f32; 3]) -> Vec3f {
    Vec3f::new(p[0] as Fp, p[1] as Fp, p[2] as Fp)
}

fn slice4_to_vec3f(p: &[f32; 4]) -> Vec3f {
    Vec3f::new(p[0] as Fp, p[1] as Fp, p[2] as Fp)
}

fn read_primitives<'a>(
    camera: &mut Camera,
    finite_primitives: &mut Vec<Primitive>,
    light_primitives: &mut Vec<Primitive>,
    buffers: &Vec<Data>,
    node: &gltf::Node,
    transformation: &Matrix4<Fp>,
    rotation: &Quaternion<Fp>,
) {
    println!("Node #{}", node.index());
    let local_transformation_slice = node.transform().matrix();
    println!(
        "Local transform good form {:#?}",
        node.transform().decomposed()
    );
    let current_rotation = UnitQuaternion::from_quaternion(Quaternion::<Fp>::new(
        node.transform().decomposed().1[3] as Fp,
        node.transform().decomposed().1[0] as Fp,
        node.transform().decomposed().1[1] as Fp,
        node.transform().decomposed().1[2] as Fp,
    ) * rotation);
    
    let local_transformation = Matrix4::from_fn(|y, x| {
        println!("y={} x={} res={}", y, x, local_transformation_slice[x][y]);
        local_transformation_slice[x][y] as Fp
    });
    // println!("local transform slice: {:#?}", local_transformation_slice);
    // dbg!(local_transformation_slice);
    // dbg!(local_transformation);
    // println!("local transform: {:#?}", local_transformation);
    let m_transformation = transformation * local_transformation;
    println!("transform: {:#?}", transformation);
    // println!("m_transform: {:#?}", m_transformation);
    if let Some(node_camera) = node.camera() {
        let Perspective(persp) = node_camera.projection() else {
            todo!()
        };
        camera.camera_fov_y = persp.yfov() as Fp;
        camera.camera_fov_x = (persp.aspect_ratio().unwrap_or(1.0) * persp.yfov()) as Fp;
        let camera_position = m_transformation * Vec4f::w();
        let camera_up = -m_transformation * Vec4f::z();
        let camera_right = m_transformation * Vec4f::y();
        let camera_forward = m_transformation * Vec4f::x();
        camera.camera_position = pp4_to_r3(camera_position);
        camera.camera_up = pp4_vector_slice(camera_right);
        camera.camera_right = pp4_vector_slice(camera_forward);
        camera.camera_forward = pp4_vector_slice(camera_up);
    }

    if let Some(mesh) = node.mesh() {
        println!("Mesh #{}", mesh.index());
        let primitive = mesh.primitives().next().unwrap();
        let reader = primitive.reader(|buffer| Some(&buffers[buffer.index()]));
        let gltfMaterial = primitive.material();
        let Some(indices) = reader.read_indices() else {
            todo!()
        };
        let indices = match indices {
            ReadIndices::U16(indices) => {
                indices.map(|x| x as usize).collect::<Vec<_>>()
            }
            ReadIndices::U8(indices) => { indices.map(|x| x as usize).collect::<Vec<_>>() }
            ReadIndices::U32(indices) => {
                indices.map(|x| x as usize).collect::<Vec<_>>()
            }
        };

        // println!("Node #{} has some {} indices!", node.index(), indices);
        let positions = reader
            .read_positions()
            .map(|x| x.collect::<Vec<_>>())
            .expect("Missing positions!");
        let maybe_normals = reader.read_normals().map(|x| x.collect::<Vec<_>>());
        for triangle in indices.chunks_exact(3) {
            assert_eq!(triangle.len(), 3);
            let to_vec3f_with_transform = |l: &[f32; 3]| {
                let point = Matrix4x1::new(l[0] as Fp, l[1] as Fp, l[2] as Fp, 1.0);
                let point = m_transformation * point;
                Vec3f::new(
                    point[0] / point[3],
                    point[1] / point[3],
                    point[2] / point[3],
                )
            };
            let a = to_vec3f_with_transform(&positions[triangle[0]]);
            let b = to_vec3f_with_transform(&positions[triangle[1]]);
            let c = to_vec3f_with_transform(&positions[triangle[2]]);
            let default_normal = (b - a).cross(&(c - a)).normalize();
            let (a_norm, b_norm, c_norm) = match maybe_normals {
                Some(ref normals) => {
                    // println!("Some normals!");
                    (
                        current_rotation.transform_vector(&slice3_to_vec3f(&normals[triangle[0]])),
                        current_rotation.transform_vector(&slice3_to_vec3f(&normals[triangle[1]])),
                        current_rotation.transform_vector(&slice3_to_vec3f(&normals[triangle[2]])),
                    )
                }
                None => {
                    println!("None normals!");
                    (default_normal, default_normal, default_normal)
                }
            };
            let object3d = Object3D {
                shape: Shape3D::Triangle {
                    a,
                    b,
                    c,
                    a_norm,
                    b_norm,
                    c_norm,
                }
                .clone(),
                position: Vec3f::new(0.0, 0.0, 0.0),
                rotation: Default::default(),
            };
            let roughness = gltfMaterial.pbr_metallic_roughness();
            // dbg!(gltfMaterial.name());
            let material = Material {
                base_color_factor: slice4_to_vec3f(&roughness.base_color_factor()),  // aka color
                metallic_factor: roughness.metallic_factor() as Fp,
                metallic_roughness: Fp::max(roughness.roughness_factor() as Fp, 0.3) as Fp,
            };
            let emission = {
                let emission = gltfMaterial.emissive_factor();
                let emission_strength = gltfMaterial.emissive_strength().unwrap_or(1.0) as Fp;
                Vec3f::new(
                    emission[0] as Fp * emission_strength,
                    emission[1] as Fp * emission_strength,
                    emission[2] as Fp * emission_strength,
                )
            };
            let primitive = Primitive {
                object3d: object3d.clone(),
                aabb: calculate_aabb_for_object(&object3d),
                material,
                ior: 1.5,
                emission,
            };
            finite_primitives.push(primitive.clone());
            if emission.norm() > EPS {
                light_primitives.push(primitive);
            }
        }
    }
    for child in node.children() {
        read_primitives(
            camera,
            finite_primitives,
            light_primitives,
            buffers,
            &child,
            &m_transformation,
            &current_rotation.into_inner(),
        );
    }
}
