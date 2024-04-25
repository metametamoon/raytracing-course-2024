mod aabb;
mod bvh;
mod distributions;
mod geometry;
mod gltf_to_scene;
mod rendering;
mod scene;
#[cfg(test)]
mod tests;

extern crate nalgebra as na;

use crate::gltf_to_scene::convert_gltf_to_scene;
use crate::rendering::render_scene;
use crate::scene::{Scene};
use env_logger::Builder;


use image::{ImageFormat, RgbImage};
use log::LevelFilter;
use std::fs;
use std::fs::File;
use std::io::Write;
use std::path::Path;
use std::time::Instant;

fn main() {
    Builder::new()
        .filter_level(LevelFilter::Debug)
        .target(env_logger::Target::Pipe(Box::new(
            File::create("out.log").unwrap(),
        )))
        .init();

    log::debug!("On creation.");
    let args: Vec<String> = std::env::args().collect();
    let input_scene = &args[1];
    let width = args[2].parse::<i32>().unwrap();
    let height = args[3].parse::<i32>().unwrap();
    let samples = args[4].parse::<i32>().unwrap();
    let output_ppm = &args[5];
    let maybe_output_png = args.get(6);

    let (gltf, buffers, _) = gltf::import(input_scene).unwrap();

    let scene = convert_gltf_to_scene(&gltf, &buffers, width, height, samples);
    // let scene = parse_file_content(file_lines);
    println!(
        "Scene finite primitives: {}, light sources: {}",
        scene.bvh_finite_primitives.primitives.len(),
        scene.bvh_light_sources.primitives.len()
    );
    let start = Instant::now();
    let rendered_scene = render_scene(&scene);
    dbg!(rendered_scene.len());
    let duration = start.elapsed();
    println!("Rendering took {:?}", duration);
    println!("Dumping to {}", output_ppm);
    let out_ppm_path: &Path = output_ppm.as_ref();
    println!("Canonic path: {:?}", out_ppm_path.canonicalize());
    let mut out_file = fs::OpenOptions::new()
        .append(true)
        .create(true)
        .open(output_ppm)
        .unwrap();
    dump_rendered_to_ppm(&scene, &rendered_scene, &mut out_file);
    if let Some(output_png) = maybe_output_png {
        let png_path = format!("{}.png", output_png);
        dump_rendered_to_png(&scene, rendered_scene, &png_path);
        println!("Image dumped to {}", png_path);
    }
}

fn dump_rendered_to_png(scene: &Scene, rendered_scene: Vec<u8>, png_path: &str) {
    let mut img = RgbImage::new(scene.width as u32, scene.height as u32);
    for x in 0..scene.width {
        for y in 0..scene.height {
            for i in 0..3 {
                img.get_pixel_mut(x as u32, y as u32).0[i] =
                    rendered_scene[(y * scene.width * 3 + x * 3) as usize + i];
            }
        }
    }
    img.save_with_format(png_path, ImageFormat::Png).unwrap();
}

fn dump_rendered_to_ppm(scene: &Scene, rendered_scene: &Vec<u8>, out_file: &mut File) {
    out_file.write_all(b"P6\n").unwrap();
    out_file
        .write_all(format!("{} {}\n", scene.width, scene.height).as_bytes())
        .unwrap();
    out_file.write_all(b"255\n").unwrap();
    out_file.write_all(rendered_scene.as_slice()).unwrap();
}
