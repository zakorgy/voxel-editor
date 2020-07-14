use cgmath::Vector3;
use crate::renderer::{Renderer, DEFAULT_MESH_COUNT};
use crate::geometry::*;
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
    DrawReleased,
    Erase,
}

pub struct Editor {
    window: winit::window::Window,
    renderer: Renderer,
    state: EditorState,
    cursor_pos_world: Vector3<f32>,
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
                    self.state = EditorState::DrawReleased;
                }
            }
        };
        if let event::WindowEvent::CursorMoved {
            position,
            ..
        } = event {
            self.cursor_pos_world = unproject(
                position.x as f32,
                position.y as f32,
                1.0 as f32,
                self.renderer.camera.model_view_mat(),
                self.renderer.camera.projection_mat(),
                self.window.inner_size(),
            );
        }

        if self.state == EditorState::ChangeView {
            self.renderer.update_view(event);
        }

        let cam_pos: Vector3<f32> = self.renderer.camera.camera.camera(0.0).position.into();
        #[cfg(feature = "debug_ray")]
        self.renderer.cursor_helper(Some(cam_pos), self.cursor_pos_world);
        let cursor_ray = Ray::new(
            cam_pos,
            cam_pos - self.cursor_pos_world,
        );

        #[cfg(feature = "debug_ray")]
        let mut closest_plane_name = "None";
        let mut closest_plane = None;
        let mut intersection_point = Vector3::new(0.0, 0.0, 0.0);
        let mesh_count = self.renderer.mesh_count as f32;
        for plane in [XY_PLANE, YZ_PLANE, XZ_PLANE].iter() {
            if let Some(point) = cursor_ray.plane_intersection(plane) {
                #[cfg(feature = "debug_ray")]
                log::debug!("{:?} intersects with mouse world position at {:?}", plane.name, point);
                if point.x <= mesh_count && point.x >= 0.0 &&
                    point.y <= mesh_count && point.y >= 0.0 &&
                    point.z <= mesh_count && point.z >= 0.0 {
                        intersection_point = point;

                        // Workaround for really small floating point coordinates can cause stuttering in the cursor movement
                        if intersection_point.x < EPSYLON {
                            intersection_point.x = 0.0;
                        }
                        if intersection_point.y < EPSYLON {
                            intersection_point.y = 0.0;
                        }
                        if intersection_point.z < EPSYLON {
                            intersection_point.z = 0.0;
                        }

                        #[cfg(feature = "debug_ray")]
                        {
                            closest_plane_name = plane.name;
                        }
                        closest_plane = Some(plane);
                        break;
                    }
                // An aletrnative way to compute closest plane for more general cases using a dist variable
                /*let dist_vec = cam_pos - point;
                let dot = dist_vec.dot(dist_vec);
                if dot < dist {
                    intersection_point = point;
                    dist = dot;
                }*/
            }
        }

        #[cfg(feature = "debug_ray")]
        log::debug!("Mouse intersects {:?} plane at {:?}", closest_plane_name, intersection_point);
        match self.state {
            EditorState::ChangeView => {
                self.renderer.update_cursor_pos(intersection_point, closest_plane);
            },
            EditorState::Draw => {
                self.renderer.update_draw_rectangle(intersection_point, closest_plane);
            },
            EditorState::DrawReleased => {
                self.state = EditorState::ChangeView;
            },
            EditorState::Erase => {
                unimplemented!()
            }
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
        let renderer = Renderer::init(surface, device, queue, sc_desc, swap_chain, DEFAULT_MESH_COUNT);
        let mut editor = Editor {
            window,
            renderer,
            state: EditorState::ChangeView,
            cursor_pos_world: Vector3::new(0.0, 0.0, 0.0),
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
