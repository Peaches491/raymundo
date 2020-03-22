extern crate image; // DMDBG: Not sure if this should really be speaking 'image' directly

use std::boxed::Box;
use std::collections::HashMap;

use crate::shape;

use crate::geometry;
use geometry::{Ray, RayHit};

pub struct Scene {
    pub lights: HashMap<String, shape::PointLight>,
    pub shapes: HashMap<String, Box<dyn shape::Shape>>,
}

impl Scene {
    pub fn new() -> Self {
        return Scene {
            lights: HashMap::new(),
            shapes: HashMap::new(),
        };
    }

    pub fn add_light(&mut self, name: &str, light: shape::PointLight) {
        self.lights.insert(name.to_string(), light);
    }

    pub fn get_light(&self, name: &str) -> Option<&shape::PointLight> {
        self.lights.get(name)
    }

    pub fn add_shape(&mut self, name: &str, shape: Box<dyn shape::Shape>) {
        self.shapes.insert(name.to_string(), shape);
    }

    pub fn get_shape(&self, name: &str) -> Option<&Box<dyn shape::Shape>> {
        self.shapes.get(name)
    }

    pub fn ray_cast(&self, ray: &Ray) -> Option<RayHit> {
        self.shapes
            .iter()
            .map(|item| item.1.ray_cast(&ray))
            .filter(|hit| hit.is_some())
            .map(|hit| hit.unwrap())
            .max_by(|lhs, rhs| {
                let lhs_norm = lhs.near.coords.norm();
                let rhs_norm = rhs.near.coords.norm();
                //lhs_norm.partial_cmp(&rhs_norm).unwrap().reverse()
                if lhs_norm > rhs_norm {
                    std::cmp::Ordering::Less
                } else if lhs_norm < rhs_norm {
                    std::cmp::Ordering::Greater
                } else {
                    std::cmp::Ordering::Equal
                }
            })
    }

    pub fn paint(&self, hit: &RayHit) -> image::Rgb<u8> {
        let n = hit.normal;
        let l = (self.get_light("light").unwrap().pose * (-hit.near))
            .coords
            .normalize();
        let n_dot_l = n.dot(&l);
        let val = num::clamp(n_dot_l * 255.0, 0.0, 255.0) as u8;

        return image::Rgb([val, val, val]);
    }
}
