use std::f64::consts::PI;

use rand::distributions::Distribution;
use rand::prelude::ThreadRng;
use rand_distr::Normal;

use crate::distributions::{SampleDistribution, VndfDistribution};
use crate::geometry::{Fp, Vec3f};
use crate::scene::Material;

fn random_unit_vec(rng: &mut ThreadRng) -> Vec3f {
    let normal_distr = Normal::new(0.0, 1.0).unwrap();

    Vec3f::new(
        normal_distr.sample(rng),
        normal_distr.sample(rng),
        normal_distr.sample(rng),
    )
    .normalize()
}

fn test_distribution(distribution: impl SampleDistribution<ThreadRng>, n: &Vec3f, v: &Vec3f, roughness: Fp) {
    let mut rng = rand::thread_rng();
    let material = Material {
        base_color_factor: Default::default(),
        metallic_factor: 0.0,
        metallic_roughness: roughness,
    };
    let point = Vec3f::default();
    let cnt = 1_000_000;
    let results = (0..cnt)
        .map(|_| distribution.pdf(&point, n, &random_unit_vec(&mut rng), v, &material))
        .collect::<Vec<_>>();
    let avg = results.iter().sum::<Fp>() / (cnt as Fp);
    let sphere_area = 4.0 * PI;
    println!("avg={}", avg * sphere_area);
    if (avg * sphere_area - 1.0) > 1.0 {
        println!("Creepy!");
    }
    assert!((avg * sphere_area - 1.0) < 0.05);
}

#[test]
fn test_vndf() {
    let z = Vec3f::z();
    let x = Vec3f::z();
    let v = (z + x).normalize();
    test_distribution(VndfDistribution{}, &z, &v, 0.04);
}

// #[test]
// fn test_cosine_distribution() {
//     test_distribution(CosineWeightedDistribution);
// }

// #[test]
// fn test_light_box_distribution_norotation() {
//     let object3d = Object3D {
//         shape: Shape3D::Box {
//             s: Vec3f::new(1.0, 2.0, 3.0),
//         },
//         position: Vec3f::new(0.0, 0.0, 4.0),
//         rotation: UnitQuaternion::default(),
//     };
//     test_distribution(DirectLightSamplingDistribution { object3d })
// }

// #[test]
// fn test_light_box_distribution() {
//     let mut rng = rand::thread_rng();
//     let rnd_rotation = UnitQuaternion::from_quaternion(Quaternion::new(
//         rng.gen(),
//         rng.gen(),
//         rng.gen(),
//         rng.gen(),
//     ));
//     let object3d = Object3D {
//         shape: Shape3D::Box {
//             s: Vec3f::new(1.0, 2.0, 3.0),
//         },
//         position: Vec3f::new(0.0, 0.0, 4.0),
//         rotation: rnd_rotation,
//     };
//     test_distribution(DirectLightSamplingDistribution { object3d })
// }


