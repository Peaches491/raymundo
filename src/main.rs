extern crate image;
extern crate log;
extern crate nalgebra as na;
extern crate num;
extern crate simple_logging;

use log::info;
use log::LevelFilter;

mod geometry;
mod graphics;
use graphics::GraphicsContext;
mod scene;
mod shape;

fn init_logging() {
    simple_logging::log_to_stderr(LevelFilter::Info);
    info!("Logging initalized!");
}

fn build_graphics_context(iso: na::Isometry3<f32>) -> GraphicsContext {
    //let scale = 0.25;
    //let scale = 0.5;
    let scale = 1.0;
    let img_width = (1024.0 * scale) as u32;
    let img_height = (1024.0 * scale) as u32;
    let imgbuf = image::RgbImage::new(img_width, img_height);

    //let aspect = img_width as f32 / img_height as f32;
    //let proj = na::Matrix4::new_perspective(aspect, 3.14 / 8.0, 0.1, 9.0);
    let size = 2.5;
    let proj = na::Orthographic3::new(-size, size, size, -size, -size, size);

    return GraphicsContext {
        tf_root: iso,
        projection: proj,
        img_width: img_width,
        img_height: img_height,
        imgbuf: imgbuf,
    };
}

fn main() {
    init_logging();

    let target = na::Isometry3::translation(0.0, 0.0, -10.0);

    let eye = na::Point3::new(0.0, 0.0, 0.0);
    let up = -na::Vector3::y();

    let iso = na::Isometry3::face_towards(&eye, &(&target * na::Point3::origin()), &up);

    let mut ctx = build_graphics_context(iso);

    let mut scene = scene::Scene::new();

    scene.add_light(shape::PointLight {
        pose: target * na::Isometry3::<f32>::translation(2.0, 1.0, 2.0),
    });

    {
        let sphere = scene.add_shape(Box::new(shape::Sphere {
            pose: target,
            radius: 1.0,
        }));

        info!("Sampling image");

        ctx.imgbuf = image::RgbImage::from_fn(ctx.img_width, ctx.img_height, |x, y| {
            let ray = ctx.unproject_point(na::Point2::new(x, y));
            match scene.ray_cast(&ray) {
                Ok(hit) => {
                    return scene.paint(&hit);
                }
                Err(_) => image::Rgb([0, 150, 200]),
            }
        });

        info!("Drawing axes");
        graphics::draw_axes(&sphere.origin(), 0.5, &mut ctx);
        //graphics::draw_axes(&light.origin(), 0.5, &mut ctx);
    }

    info!("Saving image");
    ctx.save("test.png");
}
