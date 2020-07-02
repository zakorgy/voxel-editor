mod camera;
mod editor;
mod renderer;

use editor::Editor;
use winit::{
    event_loop::EventLoop,
};

pub fn run(title: &str) {
    let event_loop = EventLoop::new();
    let mut builder = winit::window::WindowBuilder::new();
    builder = builder.with_title(title).with_inner_size(winit::dpi::LogicalSize::new(1024, 768));
    let window = builder.build(&event_loop).unwrap();
    env_logger::init();
    Editor::run_editor(event_loop, window);
}

fn main() {
    run("Voxel-editor");
}
