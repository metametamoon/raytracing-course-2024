use crate::geometry::{EPS, Fp, Vec3f};

pub fn safe_sqrt(x: Fp) -> Fp {
    Fp::max(0.0, x).sqrt()
}

pub fn chi_plus(x: Fp) -> Fp {
    if x > 0.0 {
        1.0
    } else {
        0.0
    }
}

pub fn almost_equal_vecs(x: &Vec3f, y: &Vec3f) -> bool {
    (x - y).norm() < EPS
}

pub fn almost_equal_floats(x: Fp, y: Fp) -> bool {
    (x - y).abs() < EPS
}