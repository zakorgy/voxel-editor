use crate::renderer::{Renderer, DEFAULT_MESH_RESOLUTION};
use std::time;
use winit::{
    event::{self, WindowEvent},
    event_loop::ControlFlow,
};

#[derive(Eq, PartialEq)]
enum EditorState {
    ChangeView,
    Draw,
    Erase,
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

        if self.state == EditorState::ChangeView {
            self.renderer.update_view(event);
        }
    }

    fn redraw(&mut self) {
        self.renderer.render();
    }

    pub async fn run_async(
        event_loop: winit::event_loop::EventLoop<()>,
        window: winit::window::Window
    ) {
        log::info!("Initializing the surface...");

        let instance = wgpu::Instance::new();
        let (size, surface) = unsafe {
            let size = window.inner_size();
            let surface = instance.create_surface(&window);
            (size, surface)
        };

        let adapter = instance
            .request_adapter(
                &wgpu::RequestAdapterOptions {
                    power_preference: wgpu::PowerPreference::Default,
                    compatible_surface: Some(&surface),
                },
                wgpu::BackendBit::PRIMARY,
            )
            .await
            .unwrap();

        let trace_dir = std::env::var("WGPU_TRACE");
        let (device, queue) = adapter
            .request_device(
                &wgpu::DeviceDescriptor {
                    extensions: wgpu::Extensions::empty(),
                    limits: wgpu::Limits::default(),
                },
                trace_dir.ok().as_ref().map(std::path::Path::new),
            )
            .await
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
