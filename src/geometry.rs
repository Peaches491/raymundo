extern crate nalgebra as na;

use std::fmt;

#[derive(Debug)]
pub enum CastingError {
    NoIntersection,
}

pub struct Ray {
    pub origin: na::Point3<f32>,
    pub direction: na::Vector3<f32>,
}

pub struct RayHit {
    pub near: na::Point3<f32>,
    pub far: na::Point3<f32>,
    pub normal: na::Vector3<f32>,
}

impl fmt::Display for RayHit {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(
            f,
            concat!("Normal: {}\n", "  Near: {}\n", "   Far: {}",),
            na::Point3::from(self.normal),
            self.near,
            self.far
        )
    }
}

impl fmt::Display for Ray {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(
            f,
            "[ Origin {}, Direction: ({}, {}, {}) ]",
            self.origin, self.direction.x, self.direction.y, self.direction.z,
        )
    }
}
