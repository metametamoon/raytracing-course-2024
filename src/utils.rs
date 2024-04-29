use crate::geometry::Fp;

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