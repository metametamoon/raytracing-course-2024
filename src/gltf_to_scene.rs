use crate::aabb::calculate_aabb_for_object;
use crate::geometry::{EPS, Fp, Material, Object3D, Shape3D, Vec3f, Vec4f};
use crate::scene::{Primitive, Scene};
use gltf::buffer::Data;
use gltf::mesh::util::ReadIndices;
use gltf::{Document, Gltf};
use gltf::camera::Projection::Perspective;
use na::{Matrix4, Matrix4x1, Vector4};
use crate::bvh::create_bvh_tree;

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
    for gltf_scene in gltf.scenes() {
        for node in gltf_scene.nodes() {
            read_primitives(&mut camera, &mut finite_primitives, &mut light_primitives, &buffers, &node, &default_transformation);
        }
    }
    eprintln!("here!");
    dbg!(&camera);
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
        lights: vec![],
        ray_depth: 6,
        ambient_light: Default::default(),
        samples,
        infinite_primitives: vec![],
        bvh_light_sources: create_bvh_tree(light_primitives),
    }
}

fn one_extend(v: Vec3f) -> Vec4f {
    Vector4::<Fp>::new(v.x, v.y, v.z, 1.0 as Fp)
}

fn pp4_to_r3(v: Vec4f) -> Vec3f {
    Vec3f::new(v.x / v.w, v.y / v.w, v.z / v.w)
}

fn pp4_vector_slice(v: Vec4f) -> Vec3f {
    Vec3f::new(v.x, v.y, v.z)
}

fn read_primitives<'a>(
    camera: &mut Camera,
    finite_primitives: &mut Vec<Primitive>,
    light_primitives: &mut Vec<Primitive>,
    buffers: &Vec<Data>,
    node: &gltf::Node,
    transformation: &Matrix4<Fp>,
) {
    eprintln!("Node #{}", node.index());
    let local_transformation_slice = node.transform().matrix();
    eprintln!("Local transform good form {:#?}", node.transform().decomposed());
    let local_transformation = Matrix4::from_fn(|y, x| {
        println!("y={} x={} res={}", y, x, local_transformation_slice[x][y]);
        local_transformation_slice[x][y] as Fp
    });
    // println!("local transform slice: {:#?}", local_transformation_slice);
    dbg!(local_transformation_slice);
    dbg!(local_transformation);
    // println!("local transform: {:#?}", local_transformation);
    let m_transformation = transformation * local_transformation;
    println!("transform: {:#?}", transformation);
    println!("m_transform: {:#?}", m_transformation);
    if let Some(node_camera) = node.camera(){
        let Perspective(persp) = node_camera.projection() else { todo!() };
        camera.camera_fov_y = persp.yfov() as Fp;
        camera.camera_fov_x = (persp.aspect_ratio().unwrap_or(1.0) * persp.yfov()) as Fp;
        let camera_position = m_transformation * Vec4f::w();
        let camera_up = -m_transformation * Vec4f::z();
        let camera_right = m_transformation * Vec4f::y();
        let camera_forward = m_transformation * Vec4f::x();
        camera.camera_position = pp4_to_r3(camera_position);
        camera.camera_up = pp4_vector_slice(camera_right);
        camera.camera_right = pp4_vector_slice(camera_forward);
        camera.camera_forward = pp4_vector_slice(camera_up); //right
    }

    if let Some(mesh) = node.mesh() {
        let primitive = mesh.primitives().next().unwrap();
        let reader = primitive.reader(|buffer| Some(&buffers[buffer.index()]));
        let material = primitive.material();
        let Some(indices) = reader.read_indices() else { todo!() };
        let indices = if let ReadIndices::U16(indices) = indices {
            println!("u16 indices ({})!", indices.len());
            indices
        } else {
            panic!("Only indexed rendering supported!")
        }
        .collect::<Vec<_>>();

        // println!("Node #{} has some {} indices!", node.index(), indices);
        let Some(positions) = reader.read_positions().map(|x| x.collect::<Vec<_>>()) else { todo!() };
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
            let object3d = Object3D {
                shape: Shape3D::Triangle {
                    a: to_vec3f_with_transform(&positions[triangle[0] as usize]),
                    b: to_vec3f_with_transform(&positions[triangle[1] as usize]),
                    c: to_vec3f_with_transform(&positions[triangle[2] as usize]),
                }
                .clone(),
                position: Vec3f::new(0.0, 0.0, 0.0),
                rotation: Default::default(),
            };
            let roughness = material.pbr_metallic_roughness();
            let color = {
                let base = roughness.base_color_factor();
                Vec3f::new(base[0] as Fp, base[1] as Fp, base[2] as Fp)
            };
            let material_type = {
                let base = roughness.base_color_factor();
                if base[3] < 1.0 {
                    Material::Dielectric
                } else if roughness.metallic_factor() > 0.0 {
                    Material::Metallic
                } else {
                    Material::Diffused
                }
            };
            let emission = {
                let emission = material.emissive_factor();
                let emission_strength = material.emissive_strength().unwrap_or(1.0) as Fp;
                Vec3f::new(
                    emission[0] as Fp * emission_strength,
                    emission[1] as Fp * emission_strength,
                    emission[2] as Fp * emission_strength,
                )
            };
            let primitive = Primitive {
                object3d: object3d.clone(),
                aabb: calculate_aabb_for_object(&object3d),
                color,
                material: material_type,
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
        read_primitives(camera, finite_primitives, light_primitives, buffers, &child, &m_transformation);
    }
}
