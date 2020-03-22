extern crate approx;
extern crate log;
extern crate nalgebra as na;
extern crate simple_logging;

use crate::geometry;
use geometry::{Ray, RayHit};

use std::fmt;

pub trait Shape {
    fn ray_cast(&self, ray: &Ray) -> Option<RayHit>;
    fn origin(&self) -> &na::Isometry3<f32>;
}

pub struct Plane {
    pub pose: na::Isometry3<f32>,
    pub x_dim: f32,
    pub y_dim: f32,
}

impl Shape for Plane {
    fn origin(&self) -> &na::Isometry3<f32> {
        return &self.pose;
    }

    fn ray_cast(&self, ray: &Ray) -> Option<RayHit> {
        let n = (self.pose * na::Vector3::z()).normalize();
        let l = ray.direction;
        let denom = l.dot(&(n * -1.0));
        if denom < 1.0e-6 {
            return None;
        }

        let d = self.pose * na::Point3::origin() - ray.origin;
        let t = d.dot(&(n * -1.0)) / denom;
        if t < 0.0 {
            return None;
        }

        let near_hit = ray.origin + ray.direction * t;
        Some(RayHit {
            near: near_hit,
            far: near_hit,
            normal: (self.pose * na::Vector3::z()).normalize(),
        })
    }
}

pub struct Sphere {
    pub pose: na::Isometry3<f32>,
    pub radius: f32,
}

impl Shape for Sphere {
    fn origin(&self) -> &na::Isometry3<f32> {
        return &self.pose;
    }

    fn ray_cast(&self, ray: &Ray) -> Option<RayHit> {
        // L = C - O
        let l = self.pose.translation * na::Point3::origin() - ray.origin;

        // T_ca = L dot D
        let t_ca = l.dot(&ray.direction.normalize());
        if t_ca < 0.0 {
            return None;
        }

        // d = sqrt(l^2 - T_ca^2)
        let d = (l.dot(&l) - t_ca.powi(2)).sqrt();
        if d > self.radius || d < 0.0 {
            return None;
        }

        // T_hc = sqrt(r^2 - d^2)
        let t_hc = (self.radius.powi(2) - d.powi(2)).sqrt();
        let t_0 = t_ca - t_hc;
        let t_1 = t_ca + t_hc;

        let near_hit = ray.origin + (ray.direction * t_0);
        let far_hit = ray.origin + (ray.direction * t_1);
        let normal = (near_hit.coords - self.pose.translation.vector).normalize();

        Some(RayHit {
            near: near_hit,
            far: far_hit,
            normal: normal,
        })
    }
}

impl fmt::Display for Sphere {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(
            f,
            "[Pose {}, Radius: ({})]",
            self.pose.translation * na::Point3::origin(),
            self.radius
        )
    }
}

pub struct PointLight {
    pub pose: na::Isometry3<f32>,
}

#[cfg(test)]
mod tests {
    use super::*;

    use approx::relative_eq;
    use log::info;

    fn make_ray(ray_origin: [f32; 3], ray_direction: [f32; 3]) -> Ray {
        Ray {
            origin: na::Point3::<f32>::new(ray_origin[0], ray_origin[1], ray_origin[2]),
            direction: na::Vector3::<f32>::new(
                ray_direction[0],
                ray_direction[1],
                ray_direction[2],
            ),
        }
    }

    fn make_sphere(sphere_origin: [f32; 3], radius: f32) -> Sphere {
        Sphere {
            pose: na::Isometry3::<f32>::translation(
                sphere_origin[0],
                sphere_origin[1],
                sphere_origin[2],
            ),
            radius: radius,
        }
    }

    fn make_scene(
        ray_origin: [f32; 3],
        ray_direction: [f32; 3],
        sphere_origin: [f32; 3],
        radius: f32,
    ) -> (Ray, Sphere) {
        (
            make_ray(ray_origin, ray_direction),
            make_sphere(sphere_origin, radius),
        )
    }

    fn unit_sphere() -> Sphere {
        make_sphere([0.0, 0.0, 0.0], 1.0)
    }

    #[test]
    fn intersection() {
        simple_logging::log_to_stderr(log::LevelFilter::Trace);
        // Ray coliniar with sphere center
        let (ray, sphere) = make_scene(
            [0.0, 0.0, 0.0], // Ray Origin
            [0.0, 0.0, 1.0], // Ray Direction
            [0.0, 0.0, 5.0], // Sphere Origin
            1.0,             // Sphere Radius
        );
        assert!(
            sphere.ray_cast(&ray).is_som(),
            "Ray {} should intersect Sphere {}",
            ray,
            sphere
        );
    }

    #[test]
    fn tangent_intersection() {
        // Ray along Z axis, with origin outside sphere radius
        let (ray, sphere) = make_scene(
            [0.0, 0.0, 0.0],                       // Ray Origin
            [0.0, 2.0_f32.sqrt(), 2.0_f32.sqrt()], // Ray Direction
            [0.0, 0.0, 2.0_f32.sqrt()],            // Sphere Origin
            1.0,                                   // Sphere Radius
        );
        assert!(
            sphere.ray_cast(&ray).is_some(),
            "Ray {} should intersect Sphere {}",
            ray,
            sphere
        );
    }

    #[test]
    fn ray_angle_intersection_q2() {
        // Ray along Z axis, with origin outside sphere radius
        let (ray, sphere) = make_scene(
            [0.0, 0.0, 0.0],  // Ray Origin
            [-0.2, 0.2, 1.0], // Ray Direction
            [0.0, 0.0, 3.0],  // Sphere Origin
            1.0,              // Sphere Radius
        );
        assert!(
            sphere.ray_cast(&ray).is_some(),
            "Ray {} should intersect Sphere {}",
            ray,
            sphere
        );
    }

    #[test]
    fn ray_angle_intersection_q3() {
        // Ray along Z axis, with origin outside sphere radius
        let (ray, sphere) = make_scene(
            [0.0, 0.0, 0.0],   // Ray Origin
            [-0.2, -0.2, 1.0], // Ray Direction
            [0.0, 0.0, 3.0],   // Sphere Origin
            1.0,               // Sphere Radius
        );
        assert!(
            sphere.ray_cast(&ray).is_some(),
            "Ray {} should intersect Sphere {}",
            ray,
            sphere
        );
    }

    #[test]
    fn ray_angle_intersection_q4() {
        // Ray along Z axis, with origin outside sphere radius
        let (ray, sphere) = make_scene(
            [0.0, 0.0, 0.0],  // Ray Origin
            [0.2, -0.2, 1.0], // Ray Direction
            [0.0, 0.0, 3.0],  // Sphere Origin
            1.0,              // Sphere Radius
        );
        assert!(
            sphere.ray_cast(&ray).is_some(),
            "Ray {} should intersect Sphere {}",
            ray,
            sphere
        );
    }

    #[test]
    fn translation_non_intersection() {
        // Ray along Z axis, with origin outside sphere radius
        let (ray, sphere) = make_scene(
            [2.0, 0.0, 0.0], // Ray Origin
            [0.0, 0.0, 1.0], // Ray Direction
            [0.0, 0.0, 5.0], // Sphere Origin
            1.0,             // Sphere Radius
        );
        assert!(
            !sphere.ray_cast(&ray).is_some(),
            "Ray {} should not intersect Sphere {}",
            ray,
            sphere
        );
    }

    #[test]
    fn ray_origin_inside_sphere() {
        // Ray along Z axis, with origin inside sphere
        let (ray, sphere) = make_scene(
            [0.0, 0.0, 0.0], // Ray Origin
            [0.0, 0.0, 1.0], // Ray Direction
            [0.0, 0.0, 2.0], // Sphere Origin
            3.0,             // Sphere Radius
        );
        assert!(
            sphere.ray_cast(&ray).is_some(),
            "Ray {} inside Sphere {} should intersect",
            ray,
            sphere
        );
    }

    #[test]
    fn ray_at_45_latitude_on_sphere() {
        // Ray along Z axis, with origin inside sphere
        let sin_45 = std::f32::consts::FRAC_PI_4.sin();
        let sphere_z = -10.0;
        let (ray, sphere) = make_scene(
            [sin_45, 0.0, 0.0],   // Ray Origin
            [0.0, 0.0, -1.0],     // Ray Direction
            [0.0, 0.0, sphere_z], // Sphere Origin
            1.0,                  // Sphere Radius
        );
        let hit = sphere.ray_cast(&ray).unwrap();
        let expected_near = na::Point3::new(sin_45, 0.0, sphere_z + sin_45);
        let expected_far = na::Point3::new(sin_45, 0.0, sphere_z - sin_45);
        let expected_normal = na::Vector3::new(sin_45, 0.0, sin_45).normalize();
        assert!(
            relative_eq!(hit.near, expected_near),
            concat!(
                "Ray {} should hit Sphere {} with near point:\n",
                "  Expected: {}\n",
                "    Actual: {}"
            ),
            ray,
            sphere,
            expected_near,
            hit.near,
        );
        assert!(
            relative_eq!(hit.far, expected_far),
            concat!(
                "Ray {} should hit Sphere {} with near point:\n",
                "  Expected: {}\n",
                "    Actual: {}"
            ),
            ray,
            sphere,
            expected_far,
            hit.far,
        );
        assert!(
            relative_eq!(hit.normal, expected_normal),
            concat!(
                "Ray {} should hit Sphere {} with 45 degree normal, but got:\n",
                "  Expected: {}\n",
                "    Actual: {}"
            ),
            ray,
            sphere,
            expected_normal,
            hit.normal
        );
    }

    #[test]
    fn ray_normal_straight_down() {
        // Ray along Z axis, with origin inside sphere
        let sphere = unit_sphere();
        let ray = make_ray([0.0, 0.0, 3.0], [0.0, 0.0, -1.0]);
        info!("Ray: {}", ray);
        info!("Sphere: {}", sphere);
        info!("Normal: {}", sphere.ray_cast(&ray).unwrap());
        assert!(
            sphere.ray_cast(&ray).is_some(),
            "Ray {} tangent to Sphere {} should intersect",
            ray,
            sphere
        );
        assert_eq!(
            sphere.ray_cast(&ray).unwrap().normal,
            na::Vector3::<f32>::new(0.0, 0.0, 1.0)
        );
    }

    #[test]
    fn ray_normal_at_sphere_edge() {
        let sphere = unit_sphere();
        let ray = make_ray([1.0, 0.0, 1.0], [0.0, 0.0, -1.0]);
        info!("Ray: {}", ray);
        info!("Sphere: {}", sphere);
        info!("Normal: {}", sphere.ray_cast(&ray).unwrap().normal);
        assert!(
            sphere.ray_cast(&ray).is_some(),
            "Ray {} tangent to Sphere {} should intersect",
            ray,
            sphere
        );
        assert_eq!(
            sphere.ray_cast(&ray).unwrap().normal,
            na::Vector3::<f32>::new(1.0, 0.0, 0.0)
        );
    }
}
