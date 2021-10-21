use crate::camera::CameraWrapper;
use crate::controls::EditOp;
use crate::fps::FpsCounter;
use crate::geometry::*;
use crate::renderer::{Renderer, DEFAULT_MESH_COUNT};
use crate::ui::Ui;
use crate::vertex::VoxelVertex;
use crate::voxel_manager::VoxelManager;
use cgmath::Vector3;
use futures::executor::block_on;
use iced_wgpu::wgpu;
use ron::de::from_reader;
use ron::ser::to_string;
use std::fs::File;
use std::io::prelude::*;
use std::io::BufWriter;
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

pub struct Editor {
    window: winit::window::Window,
    camera: CameraWrapper,
    voxel_manager: VoxelManager,
    renderer: Renderer,
    ui: Ui,
    state: EditorState,
    cursor_ray: Ray,
}

impl Editor {
    fn resize(&mut self, size: winit::dpi::PhysicalSize<u32>) {
        self.renderer.resize(size, &mut self.camera);
        self.ui = Ui::new(&self.window, self.renderer.device_mut())
    }

    pub fn get_model_data(&self) -> (Vec<VoxelVertex>, Vec<u32>) {
        self.voxel_manager.vertices()
    }

    fn update(&mut self, event: winit::event::WindowEvent) {
        self.ui.update(&event, self.window.scale_factor());
        if self.ui.is_cursor_over_ui {
            return;
        }
        // Don't change the view if we're editing the 3d canvas
        if let event::WindowEvent::MouseInput {
            state,
            button: event::MouseButton::Left,
            ..
        } = event
        {
            match state {
                event::ElementState::Pressed => self.state = EditorState::Edit,
                event::ElementState::Released => self.state = EditorState::EditFinished,
            }
        };
        if let event::WindowEvent::KeyboardInput {
            input:
                event::KeyboardInput {
                    virtual_keycode: keycode,
                    state: event::ElementState::Pressed,
                    ..
                },
            ..
        } = event
        {
            match keycode {
                Some(event::VirtualKeyCode::Space) => self.ui.controls().step_edit_op(),
                Some(event::VirtualKeyCode::G) => self.renderer.toggle_render_mesh(),
                _ => {}
            };
        };

        if let event::WindowEvent::CursorMoved { position, .. } = event {
            self.cursor_ray.from_cursor(
                position.x as f32,
                position.y as f32,
                &self.camera,
                self.window.inner_size(),
            );
        }

        if self.state == EditorState::ChangeView {
            let viewport_changed = self.camera.update(&event);
            if viewport_changed {
                self.renderer.update_view(&mut self.camera);
            }
        }
        #[cfg(feature = "debug_ray")]
        self.renderer
            .cursor_helper(Some(self.cursor_ray.origin), self.cursor_ray.end);

        let (erase_box, draw_box) = self.voxel_manager.get_intersection_boxes(&self.cursor_ray);
        #[cfg(feature = "debug_ray")]
        let mut closest_plane_name = "None";
        let mut closest_plane = None;
        let mut intersection_point = Vector3::new(0.0, 0.0, 0.0);
        let mesh_count = self.renderer.mesh_count() as f32;
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
                        EditOp::Refill => self.renderer.update_cursor_pos(bbox),
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
                        EditOp::Refill => self.renderer.update_draw_rectangle(bbox),
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
                        self.renderer
                            .draw_rectangle([c.r, c.g, c.b, c.a], &mut self.voxel_manager)
                    }
                    EditOp::Erase => self.renderer.erase_rectangle(&mut self.voxel_manager),
                    EditOp::Refill => {
                        let c = self.ui.controls().draw_color();
                        self.renderer
                            .fill_rectangle([c.r, c.g, c.b, c.a], &mut self.voxel_manager)
                    }
                };
                self.state = EditorState::ChangeView;
            }
        }

        if self.ui.controls().resize_editor_mesh() {
            let new_mesh_count = self.ui.controls().mesh_size();
            println!("resize_editor_mesh to {}", new_mesh_count);
            self.voxel_manager.resize(new_mesh_count as usize);
            self.renderer.resize_scene(new_mesh_count);
        }
    }

    fn redraw(&mut self) {
        let mouse_interaction = self.renderer.render(
            &mut self.ui,
            #[cfg(feature = "debug_ray")]
            &mut self.voxel_manager,
        );
        // Update the mouse cursor
        self.window
            .set_cursor_icon(iced_winit::conversion::mouse_interaction(mouse_interaction));
    }

    fn export_vertices(&self, file_path: String) -> std::io::Result<()> {
        let (vertex_data, indices) = self.get_model_data();
        let mut buffer = BufWriter::new(File::create(&file_path)?);

        const NORMALS_PER_FACES: u32 = 4;
        const NORMAL_COUNT: u32 = 6;
        const NORMALS: [[f32; 3]; NORMAL_COUNT as usize] = [
            [0.000, 0.000, -1.000],
            [0.000, -1.000, 0.000],
            [1.000, 0.000, 0.000],
            [0.000, 1.000, 0.000],
            [-1.000, 0.000, 0.000],
            [0.000, 0.000, 1.000],
        ];

        buffer.write_all(
            b"# List of vertex normals in (x,y,z) form; normals might not be unit vectors.\n",
        )?;
        for normal in NORMALS.iter() {
            buffer.write_all(
                format!("vn {:.3} {:.3} {:.3}\n", normal[0], normal[1], normal[2]).as_ref(),
            )?;
        }

        buffer.write_all(b"# List of geometric vertices, with (x, y, z [,w]) coordinates, w is optional and defaults to 1.0.\n")?;
        for vd in vertex_data.iter() {
            buffer.write_all(
                format!(
                    "v {:.3} {:.3} {:.3} 1.0\n",
                    vd.pos[0] as f32, vd.pos[1] as f32, vd.pos[2] as f32
                )
                .as_ref(),
            )?;
        }

        buffer.write_all(b"# Polygonal face element\n")?;

        for id in indices.chunks(3) {
            buffer.write_all(
                format!(
                    "f {}//{} {}//{} {}//{}\n",
                    id[0] + 1,
                    id[0] / NORMALS_PER_FACES % NORMAL_COUNT + 1,
                    id[1] + 1,
                    id[1] / NORMALS_PER_FACES % NORMAL_COUNT + 1,
                    id[2] + 1,
                    id[2] / NORMALS_PER_FACES % NORMAL_COUNT + 1
                )
                .as_ref(),
            )?;
        }

        buffer.flush()?;
        Ok(())
    }

    fn save_scene(&self, file_path: String) -> std::io::Result<()> {
        let mut buffer = BufWriter::new(File::create(&file_path)?);

        let s = to_string(&self.voxel_manager).expect("Serialization failed");

        buffer.write_all(s.as_bytes())?;

        buffer.flush()?;
        Ok(())
    }

    fn load_scene(&mut self, file_path: String) -> std::io::Result<()> {
        let f = File::open(&file_path)?;

        if let Ok(config) = from_reader(f) {
            self.voxel_manager = config;
        } else {
            log::error!("Can't load model from .ron file");
        }
        self.renderer.update_scene(&self.voxel_manager);
        Ok(())
    }

    pub fn init(window: winit::window::Window) -> Self {
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

        let mut camera = CameraWrapper::new(
            sc_desc.width as f32 / sc_desc.height as f32,
            DEFAULT_MESH_COUNT as f32,
        );

        log::info!("Initializing the Renderer...");
        let renderer = Renderer::init(
            surface,
            device,
            queue,
            sc_desc,
            swap_chain,
            DEFAULT_MESH_COUNT,
            &mut camera,
        );
        Editor {
            window,
            renderer,
            ui,
            state: EditorState::ChangeView,
            cursor_ray: Ray::new(Vector3::new(0.0, 0.0, 0.0), Vector3::new(1.0, 1.0, 1.0)),
            camera,
            voxel_manager: VoxelManager::new(DEFAULT_MESH_COUNT as usize),
        }
    }

    pub fn run(mut self, event_loop: winit::event_loop::EventLoop<()>) {
        let mut last_update_inst = time::Instant::now();
        let mut fps_counter = FpsCounter::init();

        log::info!("Entering render loop...");
        event_loop.run(move |event, _, control_flow| {
            if let Some(fps) = fps_counter.get_fps() {
                self.window
                    .set_title(&format!("Voxel-editor (FPS: {0}) {1}x{1}x{1}", fps, self.renderer.mesh_count()));
            }
            if let Some(file_path) = self.ui.controls().export_path() {
                match self.export_vertices(file_path) {
                    Err(e) => log::error!("Failed to export model! Reason: {:?}", e),
                    Ok(_) => log::info!("Model exported."),
                };
            }
            if let Some(file_path) = self.ui.controls().save_path() {
                match self.save_scene(file_path) {
                    Err(e) => log::error!("Failed to save model! Reason: {:?}", e),
                    Ok(_) => log::info!("Model saved."),
                };
            }
            if let Some(file_path) = self.ui.controls().load_path() {
                match self.load_scene(file_path) {
                    Err(e) => log::error!("Failed to load model! Reason: {:?}", e),
                    Ok(_) => log::info!("Model loaded."),
                };
            }
            match event {
                event::Event::MainEventsCleared => {
                    if last_update_inst.elapsed() > time::Duration::from_millis(16) {
                        self.ui.update_state();
                        self.window.request_redraw();
                        last_update_inst = time::Instant::now();
                    }
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
                        WindowEvent::Resized(size) => {
                            log::info!("Resizing to {:?}", size);
                            self.resize(size);
                        }
                        _ => {}
                    }
                    self.update(event);
                }
                event::Event::RedrawRequested(_) => {
                    self.redraw();
                    fps_counter.incr_frame();
                }
                _ => {}
            }
        });
    }
}
