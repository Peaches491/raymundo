extern crate image; // DMDBG: Not sure if this should really be speaking 'image' directly

use std::boxed::Box;

use crate::shape;

use crate::geometry;
use geometry::{CastingError, Ray, RayHit};

pub struct Scene {
    pub lights: std::vec::Vec<shape::PointLight>,
    pub shapes: std::vec::Vec<Box<dyn shape::Shape>>,
}

impl Scene {
    pub fn new() -> Self {
        return Scene {
            lights: vec![],
            shapes: vec![],
        };
    }

    pub fn add_light(&mut self, light: shape::PointLight) {
        self.lights.push(light);
    }

    pub fn add_shape<'a>(&'a mut self, shape: Box<dyn shape::Shape>) -> &'a Box<dyn shape::Shape> {
        self.shapes.push(shape);
        return &self.shapes[self.shapes.len() - 1];
    }

    pub fn ray_cast(&self, ray: &Ray) -> Result<RayHit, CastingError> {
        let mut result: Result<RayHit, CastingError> = Err(CastingError::NoIntersection);
        for s in &self.shapes {
            match s.ray_cast(&ray) {
                Ok(hit) => match &result {
                    Ok(scene_hit) => {
                        if hit.near.coords.norm() > scene_hit.near.coords.norm() {
                            result = Ok(hit);
                        }
                    }
                    Err(_) => {
                        result = Ok(hit);
                    }
                },
                Err(_) => {}
            }
        }
        return result;
    }

    pub fn paint(&self, hit: &RayHit) -> image::Rgb<u8> {
        let n = hit.normal;
        let l = (self.lights[0].pose * (-hit.near)).coords.normalize();
        let n_dot_l = n.dot(&l);
        let val = num::clamp(n_dot_l * 255.0, 0.0, 255.0) as u8;

        return image::Rgb([val, val, val]);
    }
}
