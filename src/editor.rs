use cgmath::{Matrix4, Vector3, Vector4, Transform};
use crate::renderer::{Renderer, DEFAULT_MESH_RESOLUTION};
use futures::executor::block_on;
use std::time;
use winit::{
    event::{self, WindowEvent},
    event_loop::ControlFlow,
};

#[derive(Eq, PartialEq)]
enum EditorState {
    ChangeView,
    Draw,
    _Erase,
}

fn unproject(
    winx: f32,
    winy: f32,
    winz: f32,
    model_view: Matrix4<f32>,
    projection: Matrix4<f32>,
    window_size: winit::dpi::PhysicalSize<u32>,
) -> Vector3<f32> {
    let matrix = (projection * model_view).inverse_transform().unwrap();

    let in_vec = Vector4::new(
        (winx / window_size.width as f32) * 2.0 - 1.0,
        ((window_size.height as f32 - winy) / window_size.height as f32) * 2.0 - 1.0,
        2.0 * winz - 1.0,
        1.0,
    );

    let mut out = matrix * in_vec;
    out.w = 1.0 / out.w;

    Vector3::new(out.x * out.w, out.y * out.w, out.z * out.w)
}

fn intersect(
    ray_vector: Vector3<f32>,
    ray_point: Vector3<f32>,
    plane_normal: Vector3<f32>,
    plane_point: Vector3<f32>,
) -> Option<Vector3<f32>> {
    use cgmath::InnerSpace;
    let denom = ray_vector.dot(plane_normal);
    if denom.abs() > 0.00001 {
        let diff = ray_point - plane_point;
        let prod1 = diff.dot(plane_normal);
        let prod2 = prod1 / denom;
        if prod2 >= 0.00001 {
            return Some(ray_point - ray_vector * prod2)
        }
    }
    None
}

pub struct Editor {
    window: winit::window::Window,
    renderer: Renderer,
    state: EditorState,
}

impl Editor {
    fn resize(&mut self, size: winit::dpi::PhysicalSize<u32>) {
        self.renderer.resize(size);
    }

    fn update(&mut self, event: winit::event::WindowEvent) {
        // Don't change the view if we're editing the 3d canvas
        if let event::WindowEvent::MouseInput {
            state,
            button: event::MouseButton::Left,
            ..
        } = event {
            match state {
                event::ElementState::Pressed => {
                    self.state = EditorState::Draw;
                }
                event::ElementState::Released => {
                    self.state = EditorState::ChangeView;
                }
            }
        };
        if let event::WindowEvent::CursorMoved {
            position,
            ..
        } = event {
            let world_pos_far = unproject(
                position.x as f32,
                position.y as f32,
                1.0 as f32,
                self.renderer.camera.model_view_mat(),
                self.renderer.camera.projection_mat(),
                self.window.inner_size(),
            );

            println!("World pos {:?}", world_pos_far);
            self.renderer.cursor_helper(None, world_pos_far);
            self.renderer.update_cursor(world_pos_far)
        }

        if self.state == EditorState::ChangeView {
            self.renderer.update_view(event);
        }
    }

    fn redraw(&mut self) {
        self.renderer.render();
    }

    pub fn run_editor(
        event_loop: winit::event_loop::EventLoop<()>,
        window: winit::window::Window
    ) {
        log::info!("Initializing the surface...");

        let instance = wgpu::Instance::new(wgpu::BackendBit::PRIMARY);
        let (size, surface) = unsafe {
            let size = window.inner_size();
            let surface = instance.create_surface(&window);
            (size, surface)
        };

        let adapter = block_on(instance
            .request_adapter(
                &wgpu::RequestAdapterOptions {
                    power_preference: wgpu::PowerPreference::Default,
                    compatible_surface: Some(&surface),
                },
                wgpu::UnsafeFeatures::disallow(),
            ))
            .unwrap();

        let trace_dir = std::env::var("WGPU_TRACE");
        let (device, queue) = block_on(adapter
            .request_device(
                &wgpu::DeviceDescriptor {
                    features: wgpu::Features::empty(),
                    limits: wgpu::Limits::default(),
                    shader_validation: true,
                },
                trace_dir.ok().as_ref().map(std::path::Path::new),
            ))
            .unwrap();

        let sc_desc = wgpu::SwapChainDescriptor {
            usage: wgpu::TextureUsage::OUTPUT_ATTACHMENT,
            format: wgpu::TextureFormat::Bgra8Unorm,
            width: size.width,
            height: size.height,
            present_mode: wgpu::PresentMode::Mailbox,
        };
        let swap_chain = device.create_swap_chain(&surface, &sc_desc);

        log::info!("Initializing the Renderer...");
        let renderer = Renderer::init(surface, device, queue, sc_desc, swap_chain, DEFAULT_MESH_RESOLUTION);
        let mut editor = Editor {
            window,
            renderer,
            state: EditorState::ChangeView,
        };

        let mut last_update_inst = time::Instant::now();

        log::info!("Entering render loop...");
        event_loop.run(move |event, _, control_flow| {
            let _ = (&instance, &adapter); // force ownership by the closure
            *control_flow = if cfg!(feature = "metal-auto-capture") {
                ControlFlow::Exit
            } else {
                ControlFlow::WaitUntil(time::Instant::now() + time::Duration::from_millis(10))
            };
            match event {
                event::Event::MainEventsCleared => {
                    if last_update_inst.elapsed() > time::Duration::from_millis(20) {
                        editor.window.request_redraw();
                        last_update_inst = time::Instant::now();
                    }
                }
                event::Event::WindowEvent {
                    event: WindowEvent::Resized(size),
                    ..
                } => {
                    log::info!("Resizing to {:?}", size);
                    editor.resize(size);
                }
                event::Event::WindowEvent { event, .. } => match event {
                    WindowEvent::KeyboardInput {
                        input:
                            event::KeyboardInput {
                                virtual_keycode: Some(event::VirtualKeyCode::Escape),
                                state: event::ElementState::Pressed,
                                ..
                            },
                        ..
                    }
                    | WindowEvent::CloseRequested => {
                        *control_flow = ControlFlow::Exit;
                    }
                    _ => {
                        editor.update(event);
                    }
                },
                event::Event::RedrawRequested(_) => {
                    editor.redraw();
                }
                _ => {}
            }
        });
    }
}
