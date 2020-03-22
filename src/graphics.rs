extern crate image;
extern crate log;
extern crate nalgebra as na;
extern crate simple_logging;

use crate::geometry;

use log::{error, info};
use std::mem::swap;

#[derive(Debug)]
pub struct GraphicsContext {
    pub tf_root: na::Isometry3<f32>,
    pub projection: na::Perspective3<f32>,
    //pub projection: na::Orthographic3<f32>,
    pub img_width: u32,
    pub img_height: u32,
    pub imgbuf: image::RgbImage,
}

impl GraphicsContext {
    pub fn unproject_point(&self, p: na::Point2<u32>) -> geometry::Ray {
        // Normalize pixel range from [0, width] t0 [-1, 1]
        let norm_px_x = (p.x as i64 - (self.img_width as i64 / 2)) as f32
            / (self.img_width as f32 / 2.0) as f32;
        let norm_px_y = (p.y as i64 - (self.img_height as i64 / 2)) as f32
            / (self.img_height as f32 / 2.0) as f32;

        // Compute two points in clip-space.
        // "ndc" = normalized device coordinates.
        let near_ndc_point = na::Point3::new(norm_px_x, norm_px_y, -1.0);
        let far_ndc_point = na::Point3::new(norm_px_x, norm_px_y, 1.0);

        // Unproject them to view-space.
        let near_view_point = self.projection.unproject_point(&near_ndc_point);
        let far_view_point = self.projection.unproject_point(&far_ndc_point);

        // Compute the view-space line parameters.
        let line_direction = far_view_point - near_view_point;

        geometry::Ray {
            origin: near_view_point,
            direction: line_direction.normalize(),
        }
    }

    pub fn project_point(&self, point: &na::Point3<f32>) -> na::Point2<i64> {
        // Project 3D point into Normalized Device Coordinates (-1, 1)
        let ndc_pt = self.projection.project_point(&point);
        // Transform to image space (0, 1).
        let img_pt = (ndc_pt + na::Vector3::new(1.0, 1.0, 1.0)) / 2.0;
        // Transform (0, 1) to (0, img_width) and (0, img_height)
        let px_x = (img_pt[0] * (self.img_width as f32)).round() as i64;
        let px_y = (img_pt[1] * (self.img_height as f32)).round() as i64;

        na::Point2::new(px_x, px_y)
    }

    pub fn put_pixel(&mut self, x: i64, y: i64, color: image::Rgb<u8>) {
        if x >= 0 && x < self.img_width as i64 && y >= 0 && y < self.img_height as i64 {
            self.put_pixel_unchecked(x, y, color);
        }
    }

    pub fn put_pixel_unchecked(&mut self, x: i64, y: i64, color: image::Rgb<u8>) {
        // NOTE: Invert the Y axis because we're not savages
        self.imgbuf.put_pixel(x as u32, self.img_height - y as u32, color);
    }

    pub fn save(&self, image_file: &str) {
        &self.imgbuf.save(image_file);
    }
}

pub fn draw_line(
    p0: &na::Point3<f32>,
    p1: &na::Point3<f32>,
    color: image::Rgb<u8>,
    context: &mut GraphicsContext,
) {
    let p0_px = context.project_point(&p0);
    let p1_px = context.project_point(&p1);

    let img_x_axis = 0 as i64..context.img_width as i64;
    let img_y_axis = 0 as i64..context.img_height as i64;

    if !(img_x_axis.contains(&p0_px[0]) && img_y_axis.contains(&p0_px[1])) {
        info!(
            "Line endpoint p0 falls outside image:\n  {}\n  {}",
            p0, p0_px
        );
        return;
    }
    if !(img_x_axis.contains(&p1_px[0]) && img_y_axis.contains(&p1_px[1])) {
        info!(
            "Line endpoint p1 falls outside image:\n  {}\n  {}",
            p1, p1_px
        );
        return;
    }

    let mut x0: i64 = p0_px[0];
    let mut y0: i64 = p0_px[1];
    let mut x1: i64 = p1_px[0];
    let mut y1: i64 = p1_px[1];
    let mut steep = false;
    // if the line is steep, we transpose the image
    if (x0 - x1).abs() < (y0 - y1).abs() {
        swap(&mut x0, &mut y0);
        swap(&mut x1, &mut y1);
        steep = true;
    }
    // make it left-to-right
    if x0 > x1 {
        swap(&mut x0, &mut x1);
        swap(&mut y0, &mut y1);
    }
    if x0 == x1 {
        x1 += 1;
    }
    for mut x in x0..x1 {
        let t = (x - x0) as f32 / ((x1 - x0) as f32);
        let mut y = ((y0 as f32) * (1.0 - t) + (y1 as f32) * t).round() as i64;
        if steep {
            // if transposed, de-transpose
            swap(&mut x, &mut y);
        }
        context.put_pixel(x, y, color);
    }
}

pub fn draw_axes(tf: &na::Isometry3<f32>, size: f32, mut context: &mut GraphicsContext) {
    let r = image::Rgb([255, 0, 0]);
    let g = image::Rgb([0, 255, 0]);
    let b = image::Rgb([0, 0, 255]);
    let unit_x_w = tf * na::Point3::new(size, 0.0, 0.0);
    let unit_y_w = tf * na::Point3::new(0.0, size, 0.0);
    let unit_z_w = tf * na::Point3::new(0.0, 0.0, size);
    draw_line(
        &(tf * na::Point3::<f32>::origin()),
        &unit_x_w,
        r,
        &mut context,
    );
    draw_line(
        &(tf * na::Point3::<f32>::origin()),
        &unit_y_w,
        g,
        &mut context,
    );
    draw_line(
        &(tf * na::Point3::<f32>::origin()),
        &unit_z_w,
        b,
        &mut context,
    );
}

pub fn _draw_circle(tf: &na::Isometry3<f32>, radius: f32, context: &mut GraphicsContext) {
    let circle_pt_count = 64;
    for idx in 0..(circle_pt_count) {
        let angle = 2.0 * ::std::f32::consts::PI * (idx as f32 / circle_pt_count as f32);
        let world_pt_d = na::Point3::new(radius * angle.cos(), radius * angle.sin(), 0.0);
        let world_pt = tf * world_pt_d;
        let px = context.project_point(&world_pt);
        if 0 < px[0]
            && px[0] < context.img_width as i64
            && 0 < px[1]
            && px[1] < context.img_height as i64
        {
            context.put_pixel(px[0] as i64, px[1] as i64, image::Rgb([255, 255, 255]));
        } else {
            error!("Pixels outside image! {}, {}", px[0], px[1]);
        }
    }
}

pub fn _draw_cube(
    tf: &na::Isometry3<f32>,
    size: f32,
    color: image::Rgb<u8>,
    mut context: &mut GraphicsContext,
) {
    let dp = size / 2.0;
    let dn = size / -2.0;
    let corners: [na::Point3<f32>; 8] = [
        tf * na::Point3::<f32>::new(dp, dp, dp), // 0
        tf * na::Point3::<f32>::new(dn, dp, dp), // 1
        tf * na::Point3::<f32>::new(dn, dn, dp), // 2
        tf * na::Point3::<f32>::new(dp, dn, dp), // 3
        tf * na::Point3::<f32>::new(dp, dp, dn), // 4
        tf * na::Point3::<f32>::new(dn, dp, dn), // 5
        tf * na::Point3::<f32>::new(dn, dn, dn), // 6
        tf * na::Point3::<f32>::new(dp, dn, dn), // 7
    ];
    // Top
    draw_line(&corners[0], &corners[1], color, &mut context);
    draw_line(&corners[1], &corners[2], color, &mut context);
    draw_line(&corners[2], &corners[3], color, &mut context);
    draw_line(&corners[3], &corners[0], color, &mut context);
    // Bottom
    draw_line(&corners[4], &corners[5], color, &mut context);
    draw_line(&corners[5], &corners[6], color, &mut context);
    draw_line(&corners[6], &corners[7], color, &mut context);
    draw_line(&corners[7], &corners[4], color, &mut context);
    // Sides
    draw_line(&corners[0], &corners[4], color, &mut context);
    draw_line(&corners[1], &corners[5], color, &mut context);
    draw_line(&corners[2], &corners[6], color, &mut context);
    draw_line(&corners[3], &corners[7], color, &mut context);
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_context() -> GraphicsContext {
        simple_logging::log_to_stderr(log::LevelFilter::Trace);
        let img_width = 1000;
        let img_height = 1000;
        let aspect = img_width as f32 / img_height as f32;
        //let proj = na::Perspective3::new(aspect, 3.14 / 2.0, 1.0, 11.0);
        let size = 3.0;
        let proj = na::Orthographic3::new(-size, size, -size, size, -size, size);
        GraphicsContext {
            tf_root: na::Isometry3::<f32>::identity(),
            projection: proj,
            img_width: img_width,
            img_height: img_height,
            imgbuf: image::RgbImage::new(img_width, img_height),
        }
    }

    #[test]
    fn project_and_unproject() {
        let ctx = make_context();
        info!("Hello, world");
        let point2 = na::Point2::<u32>::new(0, ctx.img_height / 2);
        //let point2 = na::Point2::<u32>::new(0, 0);
        let ray = ctx.unproject_point(point2);
        let distance = 0.0;
        let point3 = ray.origin + (ray.direction * distance);
        let reproj = ctx.project_point(&point3);
        let reproj = na::Point2::<u32>::new(reproj[0] as u32, reproj[1] as u32);
        info!("Ray:     {}", ray);
        info!("Point 3: {}", point3);
        info!("Reproj:  {}", reproj);
        assert_eq!(point2, reproj);
    }
}
