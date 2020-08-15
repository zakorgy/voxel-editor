use crate::geometry::*;
use crate::renderer::{Renderer, DEFAULT_MESH_COUNT};
use cgmath::Vector3;
use futures::executor::block_on;
use std::time;
use winit::{
    event::{self, WindowEvent},
    event_loop::ControlFlow,
};

#[derive(Eq, PartialEq)]
enum EditorState {
    ChangeView,
    Edit,
    EditFinished,
}

#[derive(Eq, PartialEq)]
enum EditOperation {
    Draw,
    Erase,
}

pub struct Editor {
    window: winit::window::Window,
    renderer: Renderer,
    state: EditorState,
    edit_op: EditOperation,
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
        } = event
        {
            match state {
                event::ElementState::Pressed => {
                    self.state = EditorState::Edit;
                }
                event::ElementState::Released => {
                    self.state = EditorState::EditFinished;
                }
            }
        };
        if let event::WindowEvent::CursorMoved { position, .. } = event {
            self.cursor_pos_world = unproject(
                position.x as f32,
                position.y as f32,
                1.0 as f32,
                self.renderer.camera.model_view_mat(),
                self.renderer.camera.projection_mat(),
                self.window.inner_size(),
            );
        }
        if let event::WindowEvent::KeyboardInput {
            input: event::KeyboardInput {
                virtual_keycode: Some(event::VirtualKeyCode::Space),
                state: event::ElementState::Pressed,
                ..
            },
            ..
        } = event
        {
            // Don't change edit op while drawing/erasing
            if self.state == EditorState::ChangeView {
                match self.edit_op {
                    EditOperation::Draw => self.edit_op = EditOperation::Erase,
                    EditOperation::Erase => self.edit_op = EditOperation::Draw,
                };
            }
        };

        if self.state == EditorState::ChangeView {
            self.renderer.update_view(event);
        }

        let cam_pos: Vector3<f32> = self.renderer.camera.camera.camera(0.0).position.into();
        #[cfg(feature = "debug_ray")]
        self.renderer
            .cursor_helper(Some(cam_pos), self.cursor_pos_world);
        let cursor_ray = Ray::new(cam_pos, cam_pos - self.cursor_pos_world);

        #[cfg(feature = "debug_ray")]
        let mut closest_plane_name = "None";
        let mut closest_plane = None;
        let mut intersection_point = Vector3::new(0.0, 0.0, 0.0);
        let mesh_count = self.renderer.mesh_count as f32;
        for plane in [XY_PLANE, YZ_PLANE, XZ_PLANE].iter() {
            if let Some(point) = cursor_ray.plane_intersection(plane) {
                #[cfg(feature = "debug_ray")]
                log::debug!(
                    "{:?} intersects with mouse world position at {:?}",
                    plane.name,
                    point
                );
                if point.x <= mesh_count
                    && point.x >= 0.0
                    && point.y <= mesh_count
                    && point.y >= 0.0
                    && point.z <= mesh_count
                    && point.z >= 0.0
                {
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
        log::debug!(
            "Mouse intersects {:?} plane at {:?}",
            closest_plane_name,
            intersection_point
        );
        match self.state {
            EditorState::ChangeView => {
                self.renderer
                    .update_cursor_pos(intersection_point, closest_plane);
            }
            EditorState::Edit => {
                self.renderer
                    .update_draw_rectangle(intersection_point, closest_plane);
            }
            EditorState::EditFinished => {
                match self.edit_op {
                    EditOperation::Draw => self.renderer.draw_rectangle(),
                    EditOperation::Erase => self.renderer.erase_rectangle(),
                };
                self.state = EditorState::ChangeView;
            }
        }
    }

    fn redraw(&mut self) {
        self.renderer.render();
    }

    pub fn run_editor(event_loop: winit::event_loop::EventLoop<()>, window: winit::window::Window) {
        log::info!("Initializing the surface...");

        let (size, surface) = {
            let size = window.inner_size();
            let surface = wgpu::Surface::create(&window);
            (size, surface)
        };

        let adapter = block_on(wgpu::Adapter::request(
            &wgpu::RequestAdapterOptions {
                power_preference: wgpu::PowerPreference::Default,
                compatible_surface: Some(&surface),
            },
            wgpu::BackendBit::PRIMARY,
        ))
        .unwrap();

        let (device, queue) = block_on(adapter.request_device(
            &wgpu::DeviceDescriptor {
                extensions: wgpu::Extensions {
                    anisotropic_filtering: false,
                },
                limits: wgpu::Limits::default(),
            },
        ));

        let sc_desc = wgpu::SwapChainDescriptor {
            usage: wgpu::TextureUsage::OUTPUT_ATTACHMENT,
            format: wgpu::TextureFormat::Bgra8Unorm,
            width: size.width,
            height: size.height,
            present_mode: wgpu::PresentMode::Mailbox,
        };
        let swap_chain = device.create_swap_chain(&surface, &sc_desc);

        log::info!("Initializing the Renderer...");
        let renderer = Renderer::init(
            surface,
            device,
            queue,
            sc_desc,
            swap_chain,
            DEFAULT_MESH_COUNT,
        );
        let mut editor = Editor {
            window,
            renderer,
            state: EditorState::ChangeView,
            cursor_pos_world: Vector3::new(0.0, 0.0, 0.0),
            edit_op: EditOperation::Draw,
        };

        let mut last_update_inst = time::Instant::now();

        log::info!("Entering render loop...");
        event_loop.run(move |event, _, control_flow| {
            let _ = &adapter; // force ownership by the closure
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
