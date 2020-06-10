mod renderer;

use renderer::{_Example, Example, run_async};
use winit::{
    event_loop::EventLoop,
};

pub fn run<E: Example>(title: &str) {
    let event_loop = EventLoop::new();
    let mut builder = winit::window::WindowBuilder::new();
    builder = builder.with_title(title);
    let window = builder.build(&event_loop).unwrap();
    env_logger::init();
    futures::executor::block_on(run_async::<E>(event_loop, window));
}

fn main() {
    run::<_Example>("cube");
}
