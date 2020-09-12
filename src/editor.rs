use crate::controls::EditOp;
use crate::geometry::*;
use crate::renderer::{Renderer, DEFAULT_MESH_COUNT};
use crate::ui::Ui;
use cgmath::Vector3;
use futures::executor::block_on;
use iced_wgpu::wgpu;
use std::time;
use std::fs::File;
use std::io::BufWriter;
use std::io::prelude::*;

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

pub struct Editor {
    window: winit::window::Window,
    renderer: Renderer,
    ui: Ui,
    state: EditorState,
    cursor_ray: Ray,
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
            let near_pos_world = unproject(
                position.x as f32,
                position.y as f32,
                0.1 as f32,
                self.renderer.camera.model_view_mat(),
                self.renderer.camera.projection_mat(),
                self.window.inner_size(),
            );

            let far_pos_world = unproject(
                position.x as f32,
                position.y as f32,
                1.0 as f32,
                self.renderer.camera.model_view_mat(),
                self.renderer.camera.projection_mat(),
                self.window.inner_size(),
            );
            self.cursor_ray = Ray::new(near_pos_world, far_pos_world);
        }

        if self.state == EditorState::ChangeView {
            self.renderer.update_view(event);
        }
        #[cfg(feature = "debug_ray")]
        self.renderer
            .cursor_helper(Some(self.cursor_ray.origin), self.cursor_ray.end);

        let (erase_box, draw_box) = self.renderer.voxel_manager.get_intersection_box(&self.cursor_ray);
        #[cfg(feature = "debug_ray")]
        let mut closest_plane_name = "None";
        let mut closest_plane = None;
        let mut intersection_point = Vector3::new(0.0, 0.0, 0.0);
        let mesh_count = self.renderer.mesh_count as f32;
        if erase_box.is_none() {
            for plane in [XY_PLANE, YZ_PLANE, XZ_PLANE].iter() {
                if let Some(point) = self.cursor_ray.plane_intersection(plane) {
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
                    /*let dist_vec = self.cursor_ray.origin - point;
                    let dot = dist_vec.dot(dist_vec);
                    if dot < dist {
                        intersection_point = point;
                        dist = dot;
                    }*/
                }
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
                if let Some(bbox) = erase_box {
                    match self.ui.controls().edit_op() {
                        EditOp::Draw => self.renderer.update_cursor_pos(draw_box.unwrap()),
                        EditOp::Erase => self.renderer.update_cursor_pos(bbox),
                    };
                } else {
                    self.renderer
                        .update_cursor_pos_on_plane(intersection_point, closest_plane);
                }
            }
            EditorState::Edit => {
                if let Some(bbox) = erase_box {
                    match self.ui.controls().edit_op() {
                        EditOp::Draw => self.renderer.update_draw_rectangle(draw_box.unwrap()),
                        EditOp::Erase => self.renderer.update_draw_rectangle(bbox),
                    };
                } else {
                    self.renderer
                        .update_draw_rectangle_on_plane(intersection_point, closest_plane);
                }
            }
            EditorState::EditFinished => {
                match self.ui.controls().edit_op() {
                    EditOp::Draw => {
                        let c = self.ui.controls().draw_color();
                        self.renderer.draw_rectangle([c.r, c.g, c.b, c.a])
                    }
                    EditOp::Erase => self.renderer.erase_rectangle(),
                };
                self.state = EditorState::ChangeView;
            }
        }
    }

    fn redraw(&mut self) {
        let mouse_interaction = self.renderer.render(&mut self.ui);
        // Update the mouse cursor
        self.window
            .set_cursor_icon(iced_winit::conversion::mouse_interaction(mouse_interaction));
    }

    fn save_vertices(&self, file_path: String) -> std::io::Result<()> {
        let (vertex_data, indices) = self.renderer.get_model_data();
        let mut buffer = BufWriter::new(File::create(&file_path)?);

        buffer.write_all(b"# List of geometric vertices, with (x, y, z [,w]) coordinates, w is optional and defaults to 1.0.\n")?;

        for vd in vertex_data.iter() {
            buffer.write_all(format!("v {:.3} {:.3} {:.3} 1.0\n", vd.pos[0] as f32, vd.pos[1] as f32, vd.pos[2] as f32).as_ref())?;
        }

        buffer.write_all(b"# List of vertex normals in (x,y,z) form; normals might not be unit vectors.\n")?;

        for vd in vertex_data.iter() {
            buffer.write_all(format!("vn {:.3} {:.3} {:.3}\n", vd.normal[0], vd.normal[1], vd.normal[2]).as_ref())?;
        }

        buffer.write_all(b"# Polygonal face element\n")?;

        for id in indices.chunks(3) {
            buffer.write_all(format!("f {}//{} {}//{} {}//{}\n", id[0] + 1, id[0] + 1, id[1] + 1, id[1] + 1, id[2] + 1, id[2] + 1).as_ref())?;
        }

        buffer.flush()?;
        Ok(())
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

        let (mut device, queue) = block_on(adapter.request_device(&wgpu::DeviceDescriptor {
            extensions: wgpu::Extensions {
                anisotropic_filtering: false,
            },
            limits: wgpu::Limits::default(),
        }));

        let sc_desc = wgpu::SwapChainDescriptor {
            usage: wgpu::TextureUsage::OUTPUT_ATTACHMENT,
            format: wgpu::TextureFormat::Bgra8UnormSrgb,
            width: size.width,
            height: size.height,
            present_mode: wgpu::PresentMode::Mailbox,
        };
        let swap_chain = device.create_swap_chain(&surface, &sc_desc);
        let ui = Ui::new(&window, &mut device);

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
            ui,
            state: EditorState::ChangeView,
            cursor_ray: Ray::new(Vector3::new(0.0, 0.0, 0.0), Vector3::new(1.0, 1.0, 1.0)),
        };

        let mut last_update_inst = time::Instant::now();

        log::info!("Entering render loop...");
        event_loop.run(move |event, _, control_flow| {
            if let Some(file_path) = editor.ui.controls().save_path() {
                match editor.save_vertices(file_path) {
                    Err(e) => println!("Failed to save file reason: {:?}", e),
                    Ok(_) => println!("File saved"),
                };
            }
            let _ = &adapter; // force ownership by the closure
            *control_flow = if cfg!(feature = "metal-auto-capture") {
                ControlFlow::Exit
            } else {
                ControlFlow::WaitUntil(time::Instant::now() + time::Duration::from_millis(10))
            };
            match event {
                event::Event::MainEventsCleared => {
                    if last_update_inst.elapsed() > time::Duration::from_millis(20) {
                        editor.ui.update_state();
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
                event::Event::WindowEvent { event, .. } => {
                    match event {
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
                        _ => {}
                    }
                    editor.ui.update(&event, editor.window.scale_factor());
                    editor.update(event);
                }
                event::Event::RedrawRequested(_) => {
                    editor.redraw();
                }
                _ => {}
            }
        });
    }
}
