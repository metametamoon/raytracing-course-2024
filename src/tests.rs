use crate::distributions::{
    CosineWeightedDistribution, LightSamplingDistribution, SampleDistribution,
};
use crate::geometry::{Object3D, Shape3D, Vec3f};
use na::{Quaternion, UnitQuaternion};
use rand::distributions::Distribution;
use rand::prelude::ThreadRng;
use rand::Rng;
use rand_distr::Normal;
use std::default::Default;
use std::f64::consts::PI;

fn random_unit_vec(rng: &mut ThreadRng) -> Vec3f {
    let normal_distr = Normal::new(0.0, 1.0).unwrap();

    Vec3f::new(
        normal_distr.sample(rng),
        normal_distr.sample(rng),
        normal_distr.sample(rng),
    )
    .normalize()
}

fn test_distribution(distribution: impl SampleDistribution) {
    let mut rng = rand::thread_rng();
    let normal = Vec3f::x();
    let point = Vec3f::default();
    let n = 1_000_000;
    let results = (0..n)
        .map(|_| distribution.pdf(&point, &normal, &random_unit_vec(&mut rng)))
        .collect::<Vec<_>>();
    let avg = results.iter().sum::<f64>() / (n as f64);
    let sphere_area = 4.0 * PI;
    println!("avg={}", avg * sphere_area);
    if (avg * sphere_area - 1.0) > 1.0 {
        println!("Creepy!");
    }
    assert!((avg * sphere_area - 1.0) < 0.05);
}

#[test]
fn test_cosine_distribution() {
    test_distribution(CosineWeightedDistribution);
}

#[test]
fn test_light_box_distribution_norotation() {
    let object3d = Object3D {
        shape: Shape3D::Box {
            s: Vec3f::new(1.0, 2.0, 3.0),
        },
        position: Vec3f::new(0.0, 0.0, 4.0),
        rotation: UnitQuaternion::default(),
    };
    test_distribution(LightSamplingDistribution { object3d })
}

#[test]
fn test_light_box_distribution() {
    let mut rng = rand::thread_rng();
    let rnd_rotation = UnitQuaternion::from_quaternion(Quaternion::new(
        rng.gen(),
        rng.gen(),
        rng.gen(),
        rng.gen(),
    ));
    let object3d = Object3D {
        shape: Shape3D::Box {
            s: Vec3f::new(1.0, 2.0, 3.0),
        },
        position: Vec3f::new(0.0, 0.0, 4.0),
        rotation: rnd_rotation,
    };
    test_distribution(LightSamplingDistribution { object3d })
}

#[test]
fn test_light_ellipsoid_distribution_norotation() {
    let object3d = Object3D {
        shape: Shape3D::Ellipsoid {
            r: Vec3f::new(1.0, 2.0, 3.0),
        },
        position: Vec3f::new(0.0, 0.0, 4.0),
        rotation: Default::default(),
    };
    test_distribution(LightSamplingDistribution { object3d })
}

#[test]
fn test_light_ellipsoid_distribution() {
    let mut rng = rand::thread_rng();
    let object3d = Object3D {
        shape: Shape3D::Ellipsoid {
            r: Vec3f::new(1.0, 2.0, 3.0),
        },
        position: Vec3f::new(0.0, 0.0, 4.0),
        rotation: UnitQuaternion::from_quaternion(Quaternion::new(
            rng.gen(),
            rng.gen(),
            rng.gen(),
            rng.gen(),
        )),
    };
    test_distribution(LightSamplingDistribution { object3d })
}
