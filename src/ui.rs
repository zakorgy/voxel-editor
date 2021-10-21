use crate::controls::Controls;
use iced_wgpu::{wgpu, Backend, Renderer, Settings, Viewport};
use iced_winit::{conversion, mouse::Interaction, program, winit, Debug, Size};
use winit::{
    dpi::PhysicalPosition,
    event::{ModifiersState, WindowEvent},
    window::Window,
};

pub struct Ui {
    renderer: Renderer,
    state: program::State<Controls>,
    debug: Debug,
    viewport: Viewport,
    cursor_position: PhysicalPosition<f64>,
    modifiers: ModifiersState,
    width: u16,
    pub is_cursor_over_ui: bool,
}

impl Ui {
    pub fn new(window: &Window, device: &mut wgpu::Device) -> Ui {
        let width = 150;
        let physical_size = window.inner_size();
        let viewport = Viewport::with_physical_size(
            Size::new(physical_size.width, physical_size.height),
            window.scale_factor(),
        );
        let cursor_position = PhysicalPosition::new(-1.0, -1.0);
        let modifiers = ModifiersState::default();

        let controls = Controls::new(width);
        let mut debug = Debug::new();
        let mut renderer = Renderer::new(Backend::new(device, Settings::default()));

        let state = program::State::new(
            controls,
            viewport.logical_size(),
            conversion::cursor_position(cursor_position, viewport.scale_factor()),
            &mut renderer,
            &mut debug,
        );

        Ui {
            renderer,
            state,
            debug,
            viewport,
            cursor_position,
            modifiers,
            width,
            is_cursor_over_ui: false,
        }
    }

    pub fn controls(&self) -> &Controls {
        &self.state.program()
    }

    pub fn update_state(&mut self) {
        if !self.state.is_queue_empty() {
            self.state.update(
                self.viewport.logical_size(),
                conversion::cursor_position(self.cursor_position, self.viewport.scale_factor()),
                None,
                &mut self.renderer,
                &mut self.debug,
            );
        }
    }

    pub fn update(&mut self, event: &WindowEvent, scale_factor: f64) {
        if let Some(event) =
            iced_winit::conversion::window_event(&event, scale_factor, self.modifiers)
        {
            self.state.queue_event(event);
        }
        match *event {
            WindowEvent::CursorMoved { position, .. } => {
                if position.x > self.width as f64 {
                    self.is_cursor_over_ui = false;
                } else {
                    self.is_cursor_over_ui = true;
                }
                self.cursor_position = position;
            }
            WindowEvent::ModifiersChanged(new_modifiers) => {
                self.modifiers = new_modifiers;
            }
            WindowEvent::Resized(new_size) => {
                self.viewport = Viewport::with_physical_size(
                    Size::new(new_size.width, new_size.height),
                    scale_factor,
                );
            }
            _ => {}
        };
    }

    pub fn draw(
        &mut self,
        device: &mut wgpu::Device,
        encoder: &mut wgpu::CommandEncoder,
        view: &wgpu::TextureView,
    ) -> Interaction {
        self.renderer.backend_mut().draw(
            device,
            encoder,
            view,
            &self.viewport,
            self.state.primitive(),
            &self.debug.overlay(),
        )
    }
}

pub fn build_ui_pipeline(device: &wgpu::Device) -> wgpu::RenderPipeline {
    let vs = include_bytes!("../shaders/ui.vert.spv");
    let fs = include_bytes!("../shaders/ui.frag.spv");

    let vs_module =
        device.create_shader_module(&wgpu::read_spirv(std::io::Cursor::new(&vs[..])).unwrap());

    let fs_module =
        device.create_shader_module(&wgpu::read_spirv(std::io::Cursor::new(&fs[..])).unwrap());

    let pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
        bind_group_layouts: &[],
    });

    let pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
        layout: &pipeline_layout,
        vertex_stage: wgpu::ProgrammableStageDescriptor {
            module: &vs_module,
            entry_point: "main",
        },
        fragment_stage: Some(wgpu::ProgrammableStageDescriptor {
            module: &fs_module,
            entry_point: "main",
        }),
        rasterization_state: Some(wgpu::RasterizationStateDescriptor {
            front_face: wgpu::FrontFace::Ccw,
            cull_mode: wgpu::CullMode::None,
            depth_bias: 0,
            depth_bias_slope_scale: 0.0,
            depth_bias_clamp: 0.0,
        }),
        primitive_topology: wgpu::PrimitiveTopology::TriangleList,
        color_states: &[wgpu::ColorStateDescriptor {
            format: wgpu::TextureFormat::Bgra8UnormSrgb,
            color_blend: wgpu::BlendDescriptor::REPLACE,
            alpha_blend: wgpu::BlendDescriptor::REPLACE,
            write_mask: wgpu::ColorWrite::ALL,
        }],
        depth_stencil_state: None,
        vertex_state: wgpu::VertexStateDescriptor {
            index_format: wgpu::IndexFormat::Uint16,
            vertex_buffers: &[],
        },
        sample_count: 1,
        sample_mask: !0,
        alpha_to_coverage_enabled: false,
    });

    pipeline
}
