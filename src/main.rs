#![feature(iter_partition_in_place)]

mod aabb;
mod bvh;
mod distributions;
mod geometry;
mod rendering;
mod scene;
#[cfg(test)]
mod tests;
extern crate nalgebra as na;

use crate::rendering::render_scene;
use crate::scene::{parse_file_content, Scene};
use env_logger::Builder;
use image::{ImageFormat, RgbImage};
use log::LevelFilter;
use std::fs;
use std::fs::File;
use std::io::Write;
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
    let file_string = fs::read_to_string(&args[1]).expect("Failed opening file");
    let file_lines = file_string
        .split('\n')
        .map(|x| x.trim())
        .collect::<Vec<&str>>();
    let scene = parse_file_content(file_lines);
    println!("{:#?}", scene);
    let start = Instant::now();
    let rendered_scene = render_scene(&scene);
    dbg!(rendered_scene.len());
    let duration = start.elapsed();
    println!("Rendering took {:?}", duration);
    let mut out_file = fs::OpenOptions::new()
        .append(true)
        .create(true)
        .open(&args[2])
        .unwrap();
    dump_rendered_to_ppm(&scene, &rendered_scene, &mut out_file);
    if args.len() >= 4 {
        let png_path = format!("{}.png", args[3]);
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
    out_file.write(b"P6\n").unwrap();
    out_file
        .write(format!("{} {}\n", scene.width, scene.height).as_bytes())
        .unwrap();
    out_file.write(b"255\n").unwrap();
    out_file.write(rendered_scene.as_slice()).unwrap();
}
