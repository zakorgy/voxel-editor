use crate::camera::CameraWrapper;
use crate::geometry::*;
use crate::light::*;
use crate::ui::{build_ui_pipeline, Ui};
use crate::voxel_manager::VoxelManager;
use cgmath;
use iced_wgpu::wgpu;
use iced_winit::mouse::Interaction;

pub const DEFAULT_MESH_COUNT: u16 = 32;
const SAMPLE_COUNT: u32 = 4;

const DEPTH_FORMAT: wgpu::TextureFormat = wgpu::TextureFormat::Depth32Float;

const RED: [f32; 4] = [1.0, 0.0, 0.0, 1.0];
const HALF_ALPHA_RED: [f32; 4] = [1.0, 0.0, 0.0, 0.2];
const GREEN: [f32; 4] = [0.0, 1.0, 0.0, 1.0];
const BLUE: [f32; 4] = [0.0, 0.0, 1.0, 1.0];
const TRANSPARENT: [f32; 4] = [0.0, 0.0, 0.0, 0.0];

use bytemuck::{Pod, Zeroable};

#[derive(Clone, Copy)]
pub struct Vertex {
    _pos: [f32; 3],
    _col: [f32; 4],
}

#[derive(Clone, Copy)]
pub struct VoxelVertex {
    pub pos: [f32; 3],
    _col: [f32; 4],
    pub normal: [f32; 3],
}

unsafe impl Pod for Vertex {}
unsafe impl Zeroable for Vertex {}

unsafe impl Pod for VoxelVertex {}
unsafe impl Zeroable for VoxelVertex {}

fn white_vertex(pos: [f32; 3]) -> Vertex {
    vertex(pos, [1.0; 4])
}

fn vertex(pos: [f32; 3], col: [f32; 4]) -> Vertex {
    Vertex {
        _pos: [pos[0], pos[1], pos[2]],
        _col: col,
    }
}

fn voxel_vertex(pos: [f32; 3], col: [f32; 4], normal: [f32; 3]) -> VoxelVertex {
    VoxelVertex {
        pos: [pos[0], pos[1], pos[2]],
        _col: col,
        normal,
    }
}

fn generate_mesh_vertices(meshes: u16) -> (Vec<Vertex>, Vec<u16>) {
    let mut vertex_data = Vec::new();
    let mut index_data: Vec<u16> = Vec::new();
    let mesh_count = meshes as f32;

    // X axis
    vertex_data.push(vertex([0.0, 0.0, 0.0], RED));
    index_data.push((vertex_data.len() - 1) as u16);
    vertex_data.push(vertex([mesh_count, 0.0, 0.0], RED));
    index_data.push((vertex_data.len() - 1) as u16);

    // Y axis
    vertex_data.push(vertex([0.0, 0.0, 0.0], GREEN));
    index_data.push((vertex_data.len() - 1) as u16);
    vertex_data.push(vertex([0.0, mesh_count, 0.0], GREEN));
    index_data.push((vertex_data.len() - 1) as u16);

    // Z axis
    vertex_data.push(vertex([0.0, 0.0, 0.0], BLUE));
    index_data.push((vertex_data.len() - 1) as u16);
    vertex_data.push(vertex([0.0, 0.0, mesh_count], BLUE));
    index_data.push((vertex_data.len() - 1) as u16);

    for i in 1..(meshes + 1) {
        // back
        vertex_data.push(white_vertex([0.0, 0.0 + i as f32, 0.0]));
        index_data.push((vertex_data.len() - 1) as u16);
        vertex_data.push(white_vertex([mesh_count, 0.0 + i as f32, 0.0]));
        index_data.push((vertex_data.len() - 1) as u16);

        vertex_data.push(white_vertex([0.0 + i as f32, 0.0, 0.0]));
        index_data.push((vertex_data.len() - 1) as u16);
        vertex_data.push(white_vertex([0.0 + i as f32, mesh_count, 0.0]));
        index_data.push((vertex_data.len() - 1) as u16);

        // left
        vertex_data.push(white_vertex([0.0, 0.0 + i as f32, 0.0]));
        index_data.push((vertex_data.len() - 1) as u16);
        vertex_data.push(white_vertex([0.0, 0.0 + i as f32, mesh_count]));
        index_data.push((vertex_data.len() - 1) as u16);

        vertex_data.push(white_vertex([0.0, 0.0, 0.0 + i as f32]));
        index_data.push((vertex_data.len() - 1) as u16);
        vertex_data.push(white_vertex([0.0, mesh_count, 0.0 + i as f32]));
        index_data.push((vertex_data.len() - 1) as u16);

        // bottom
        vertex_data.push(white_vertex([0.0 + i as f32, 0.0, 0.0]));
        index_data.push((vertex_data.len() - 1) as u16);
        vertex_data.push(white_vertex([0.0 + i as f32, 0.0, mesh_count]));
        index_data.push((vertex_data.len() - 1) as u16);

        vertex_data.push(white_vertex([0.0, 0.0, 0.0 + i as f32]));
        index_data.push((vertex_data.len() - 1) as u16);
        vertex_data.push(white_vertex([mesh_count, 0.0, 0.0 + i as f32]));
        index_data.push((vertex_data.len() - 1) as u16);
    }

    // placeholder for cursor debug line
    if cfg!(feature = "debug_ray") {
        vertex_data.push(vertex([0.0, 0.0, 0.0], TRANSPARENT));
        index_data.push((vertex_data.len() - 1) as u16);
        vertex_data.push(vertex([0.0, 0.0, 0.0], TRANSPARENT));
        index_data.push((vertex_data.len() - 1) as u16);
    }

    (vertex_data, index_data)
}

fn create_texture_view(
    device: &wgpu::Device,
    sc_desc: &wgpu::SwapChainDescriptor,
    sample_count: u32,
    format: wgpu::TextureFormat,
    label: Option<&'static str>,
) -> wgpu::TextureView {
    let size = wgpu::Extent3d {
        width: sc_desc.width,
        height: sc_desc.height,
        depth: 1,
    };
    let texture_descriptor = &wgpu::TextureDescriptor {
        size,
        mip_level_count: 1,
        array_layer_count: 1,
        sample_count,
        dimension: wgpu::TextureDimension::D2,
        format,
        usage: wgpu::TextureUsage::OUTPUT_ATTACHMENT,
        label,
    };

    device
        .create_texture(texture_descriptor)
        .create_default_view()
}

impl BoundingBox {
    fn vertices(&self) -> Vec<Vertex> {
        let mut vertex_data = Vec::new();
        let corner_points = self.corner_points();
        let color = self.color;

        // back
        /*0*/
        vertex_data.push(vertex(corner_points[0].into(), color));
        /*1*/
        vertex_data.push(vertex(corner_points[3].into(), color));
        /*2*/
        vertex_data.push(vertex(corner_points[2].into(), color));
        /*3*/
        vertex_data.push(vertex(corner_points[1].into(), color));

        // bottom
        /*4*/
        vertex_data.push(vertex(corner_points[1].into(), color));
        /*5*/
        vertex_data.push(vertex(corner_points[5].into(), color));
        /*6*/
        vertex_data.push(vertex(corner_points[4].into(), color));
        /*7*/
        vertex_data.push(vertex(corner_points[0].into(), color));

        // right
        /*9*/
        vertex_data.push(vertex(corner_points[2].into(), color));
        /*8*/
        vertex_data.push(vertex(corner_points[6].into(), color));
        /*10*/
        vertex_data.push(vertex(corner_points[5].into(), color));
        /*11*/
        vertex_data.push(vertex(corner_points[1].into(), color));

        // top
        /*12*/
        vertex_data.push(vertex(corner_points[3].into(), color));
        /*13*/
        vertex_data.push(vertex(corner_points[7].into(), color));
        /*14*/
        vertex_data.push(vertex(corner_points[6].into(), color));
        /*15*/
        vertex_data.push(vertex(corner_points[2].into(), color));

        // left
        /*16*/
        vertex_data.push(vertex(corner_points[3].into(), color));
        /*17*/
        vertex_data.push(vertex(corner_points[0].into(), color));
        /*18*/
        vertex_data.push(vertex(corner_points[4].into(), color));
        /*19*/
        vertex_data.push(vertex(corner_points[7].into(), color));

        // front
        /*20*/
        vertex_data.push(vertex(corner_points[4].into(), color));
        /*21*/
        vertex_data.push(vertex(corner_points[5].into(), color));
        /*22*/
        vertex_data.push(vertex(corner_points[6].into(), color));
        /*23*/
        vertex_data.push(vertex(corner_points[7].into(), color));

        vertex_data
    }

    fn voxel_vertices(&self) -> Vec<VoxelVertex> {
        let mut vertex_data = Vec::new();
        let corner_points = self.corner_points();
        let color = self.color;

        // back
        /*0*/
        vertex_data.push(voxel_vertex(
            corner_points[0].into(),
            color,
            [0.0, 0.0, -1.0],
        ));
        /*1*/
        vertex_data.push(voxel_vertex(
            corner_points[3].into(),
            color,
            [0.0, 0.0, -1.0],
        ));
        /*2*/
        vertex_data.push(voxel_vertex(
            corner_points[2].into(),
            color,
            [0.0, 0.0, -1.0],
        ));
        /*3*/
        vertex_data.push(voxel_vertex(
            corner_points[1].into(),
            color,
            [0.0, 0.0, -1.0],
        ));

        // bottom
        /*4*/
        vertex_data.push(voxel_vertex(
            corner_points[1].into(),
            color,
            [0.0, -1.0, 0.0],
        ));
        /*5*/
        vertex_data.push(voxel_vertex(
            corner_points[5].into(),
            color,
            [0.0, -1.0, 0.0],
        ));
        /*6*/
        vertex_data.push(voxel_vertex(
            corner_points[4].into(),
            color,
            [0.0, -1.0, 0.0],
        ));
        /*7*/
        vertex_data.push(voxel_vertex(
            corner_points[0].into(),
            color,
            [0.0, -1.0, 0.0],
        ));

        // right
        /*9*/
        vertex_data.push(voxel_vertex(
            corner_points[2].into(),
            color,
            [1.0, 0.0, 0.0],
        ));
        /*8*/
        vertex_data.push(voxel_vertex(
            corner_points[6].into(),
            color,
            [1.0, 0.0, 0.0],
        ));
        /*10*/
        vertex_data.push(voxel_vertex(
            corner_points[5].into(),
            color,
            [1.0, 0.0, 0.0],
        ));
        /*11*/
        vertex_data.push(voxel_vertex(
            corner_points[1].into(),
            color,
            [1.0, 0.0, 0.0],
        ));

        // top
        /*12*/
        vertex_data.push(voxel_vertex(
            corner_points[3].into(),
            color,
            [0.0, 1.0, 0.0],
        ));
        /*13*/
        vertex_data.push(voxel_vertex(
            corner_points[7].into(),
            color,
            [0.0, 1.0, 0.0],
        ));
        /*14*/
        vertex_data.push(voxel_vertex(
            corner_points[6].into(),
            color,
            [0.0, 1.0, 0.0],
        ));
        /*15*/
        vertex_data.push(voxel_vertex(
            corner_points[2].into(),
            color,
            [0.0, 1.0, 0.0],
        ));

        // left
        /*16*/
        vertex_data.push(voxel_vertex(
            corner_points[3].into(),
            color,
            [-1.0, 0.0, 0.0],
        ));
        /*17*/
        vertex_data.push(voxel_vertex(
            corner_points[0].into(),
            color,
            [-1.0, 0.0, 0.0],
        ));
        /*18*/
        vertex_data.push(voxel_vertex(
            corner_points[4].into(),
            color,
            [-1.0, 0.0, 0.0],
        ));
        /*19*/
        vertex_data.push(voxel_vertex(
            corner_points[7].into(),
            color,
            [-1.0, 0.0, 0.0],
        ));

        // front
        /*20*/
        vertex_data.push(voxel_vertex(
            corner_points[4].into(),
            color,
            [0.0, 0.0, 1.0],
        ));
        /*21*/
        vertex_data.push(voxel_vertex(
            corner_points[5].into(),
            color,
            [0.0, 0.0, 1.0],
        ));
        /*22*/
        vertex_data.push(voxel_vertex(
            corner_points[6].into(),
            color,
            [0.0, 0.0, 1.0],
        ));
        /*23*/
        vertex_data.push(voxel_vertex(
            corner_points[7].into(),
            color,
            [0.0, 0.0, 1.0],
        ));

        vertex_data
    }
}

impl VoxelManager {
    pub fn vertices(&self) -> (Vec<VoxelVertex>, Vec<u32>) {
        let mut vertex_data = Vec::new();
        let mut index_data = Vec::new();
        let mut step;
        let mut idx;
        let mut bbox;
        for x in 0..self.extent {
            for y in 0..self.extent {
                for z in 0..self.extent {
                    if let Some(desc) = self.boxes[x][y][z] {
                        idx = vertex_data.len() as u32;
                        for i in 0..6 {
                            step = 4 * i;
                            index_data.extend_from_slice(&[
                                idx + step,
                                idx + 1 + step,
                                idx + 2 + step,
                                idx + 2 + step,
                                idx + 3 + step,
                                idx + step,
                            ]);
                        }
                        bbox = BoundingBox::new(
                            cgmath::Vector3::new(x as f32, y as f32, z as f32),
                            cgmath::Vector3::new(1.0, 1.0, 1.0),
                            desc.color,
                        );
                        vertex_data.append(&mut bbox.voxel_vertices());
                    }
                }
            }
        }
        (vertex_data, index_data)
    }
}

fn generate_cursor_vertices(bbox: &BoundingBox) -> (Vec<Vertex>, Vec<u16>) {
    let index_data: Vec<u16> = vec![
        0, 1, 2, 2, 3, 0, 4, 5, 6, 6, 7, 4, 8, 9, 10, 10, 11, 8, 12, 13, 14, 14, 15, 12, 16, 17,
        18, 18, 19, 16, 20, 21, 22, 22, 23, 20,
    ];
    (bbox.vertices(), index_data)
}

struct Pipeline {
    bind_group: wgpu::BindGroup,
    pipeline: wgpu::RenderPipeline,
    vertex_buf: wgpu::Buffer,
    index_buf: wgpu::Buffer,
    index_count: usize,
}

impl Pipeline {
    fn draw<'a>(&'a mut self, render_pass: &mut wgpu::RenderPass<'a>) {
        render_pass.set_pipeline(&self.pipeline);
        render_pass.set_bind_group(0, &self.bind_group, &[]);
        render_pass.set_index_buffer(&self.index_buf, 0, 0);
        render_pass.set_vertex_buffer(0, &self.vertex_buf, 0, 0);
        render_pass.draw_indexed(0..self.index_count as u32, 0, 0..1);
    }
}

pub struct Renderer {
    pub camera: CameraWrapper,
    surface: wgpu::Surface,
    pub device: wgpu::Device,
    pub queue: wgpu::Queue,
    sc_desc: wgpu::SwapChainDescriptor,
    pub swap_chain: wgpu::SwapChain,
    depth_buffer: wgpu::TextureView,
    multisampled_framebuffer: wgpu::TextureView,
    mvp_buf: wgpu::Buffer,
    light_uniform_buf: wgpu::Buffer,
    command_buffers: Vec<wgpu::CommandBuffer>,
    mesh_pipeline: Pipeline,
    render_cursor: bool,
    cursor_pipeline: Pipeline,
    voxel_pipeline: Pipeline,
    ui_pipeline: wgpu::RenderPipeline,
    cursor_cube: BoundingBox,
    draw_cube: Option<BoundingBox>,
    pub mesh_count: u16,
    pub voxel_manager: VoxelManager,
    light: Light,
    lights_are_dirty: bool,
}

impl Renderer {
    pub fn init(
        surface: wgpu::Surface,
        device: wgpu::Device,
        queue: wgpu::Queue,
        sc_desc: wgpu::SwapChainDescriptor,
        swap_chain: wgpu::SwapChain,
        mesh_count: u16,
    ) -> Self {
        use std::mem;

        // Create the vertex and index buffers
        let vertex_size = mem::size_of::<Vertex>();

        //****************************** Setting up mesh pipeline ******************************
        let (vertex_data, mesh_index_data) = generate_mesh_vertices(mesh_count);

        let vertex_buf_mesh = device.create_buffer_with_data(
            bytemuck::cast_slice(&vertex_data),
            wgpu::BufferUsage::VERTEX | wgpu::BufferUsage::COPY_DST,
        );

        let index_buf_mesh = device.create_buffer_with_data(
            bytemuck::cast_slice(&mesh_index_data),
            wgpu::BufferUsage::INDEX | wgpu::BufferUsage::COPY_DST,
        );

        // Create pipeline layout
        let bind_group_layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: None,
            bindings: &[wgpu::BindGroupLayoutEntry {
                binding: 0,
                visibility: wgpu::ShaderStage::VERTEX,
                ty: wgpu::BindingType::UniformBuffer { dynamic: false },
            }],
        });
        let pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            bind_group_layouts: &[&bind_group_layout],
        });

        let mut camera = CameraWrapper::new(
            sc_desc.width as f32 / sc_desc.height as f32,
            mesh_count as f32,
        );

        let matrices = camera.mvp_matrices(sc_desc.width as f32 / sc_desc.height as f32);
        let matrices_ref = matrices.as_ref();
        let uniform_buf = device.create_buffer_with_data(
            bytemuck::cast_slice(matrices_ref),
            wgpu::BufferUsage::UNIFORM | wgpu::BufferUsage::COPY_DST,
        );
        let uniform_buf_size = 3 * mem::size_of::<[f32; 16]>() as u64;

        // Create bind group
        let mesh_bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
            layout: &bind_group_layout,
            bindings: &[wgpu::Binding {
                binding: 0,
                resource: wgpu::BindingResource::Buffer {
                    buffer: &uniform_buf,
                    range: 0..uniform_buf_size,
                },
            }],
            label: None,
        });

        let vs_bytes = include_bytes!("../shaders/mesh.vert.spv");
        let fs_bytes = include_bytes!("../shaders/mesh.frag.spv");

        // Create the mesh rendering pipeline
        let vs_module = device
            .create_shader_module(&wgpu::read_spirv(std::io::Cursor::new(&vs_bytes[..])).unwrap());
        let fs_module = device
            .create_shader_module(&wgpu::read_spirv(std::io::Cursor::new(&fs_bytes[..])).unwrap());

        let mesh_pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
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
            primitive_topology: wgpu::PrimitiveTopology::LineList,
            color_states: &[wgpu::ColorStateDescriptor {
                format: sc_desc.format,
                color_blend: wgpu::BlendDescriptor::REPLACE,
                alpha_blend: wgpu::BlendDescriptor::REPLACE,
                write_mask: wgpu::ColorWrite::ALL,
            }],
            depth_stencil_state: Some(wgpu::DepthStencilStateDescriptor {
                format: DEPTH_FORMAT,
                depth_write_enabled: true,
                depth_compare: wgpu::CompareFunction::Less,
                stencil_front: wgpu::StencilStateFaceDescriptor::IGNORE,
                stencil_back: wgpu::StencilStateFaceDescriptor::IGNORE,
                stencil_read_mask: 0,
                stencil_write_mask: 0,
            }),
            vertex_state: wgpu::VertexStateDescriptor {
                index_format: wgpu::IndexFormat::Uint16,
                vertex_buffers: &[wgpu::VertexBufferDescriptor {
                    stride: vertex_size as wgpu::BufferAddress,
                    step_mode: wgpu::InputStepMode::Vertex,
                    attributes: &[
                        // Position
                        wgpu::VertexAttributeDescriptor {
                            format: wgpu::VertexFormat::Float3,
                            offset: 0,
                            shader_location: 0,
                        },
                        // Color
                        wgpu::VertexAttributeDescriptor {
                            format: wgpu::VertexFormat::Float4,
                            offset: 3 * 4,
                            shader_location: 1,
                        },
                    ],
                }],
            },
            sample_count: SAMPLE_COUNT,
            sample_mask: !0,
            alpha_to_coverage_enabled: false,
        });

        //****************************** Setting up cursor pipeline ******************************
        let cursor_cube = BoundingBox::new(
            cgmath::Vector3::new(0.0, 0.0, 0.0),
            XY_PLANE.left + XY_PLANE.down + XY_PLANE.normal,
            HALF_ALPHA_RED.into(),
        );
        let (vertex_data, cursor_index_data) = generate_cursor_vertices(&cursor_cube);

        let vertex_buf_cursor = device.create_buffer_with_data(
            bytemuck::cast_slice(&vertex_data),
            wgpu::BufferUsage::VERTEX | wgpu::BufferUsage::COPY_DST,
        );

        let index_buf_cursor = device.create_buffer_with_data(
            bytemuck::cast_slice(&cursor_index_data),
            wgpu::BufferUsage::INDEX | wgpu::BufferUsage::COPY_DST,
        );

        // Create pipeline layout
        let bind_group_layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: None,
            bindings: &[wgpu::BindGroupLayoutEntry {
                binding: 0,
                visibility: wgpu::ShaderStage::VERTEX,
                ty: wgpu::BindingType::UniformBuffer { dynamic: false },
            }],
        });
        let pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            bind_group_layouts: &[&bind_group_layout],
        });

        // Create bind group
        let cursor_bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
            layout: &bind_group_layout,
            bindings: &[wgpu::Binding {
                binding: 0,
                resource: wgpu::BindingResource::Buffer {
                    buffer: &uniform_buf,
                    range: 0..uniform_buf_size,
                },
            }],
            label: None,
        });

        // Create the cursor rendering pipeline

        let cursor_pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
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
                format: sc_desc.format,
                color_blend: wgpu::BlendDescriptor {
                    src_factor: wgpu::BlendFactor::SrcAlpha,
                    dst_factor: wgpu::BlendFactor::OneMinusSrcAlpha,
                    operation: wgpu::BlendOperation::Add,
                },
                alpha_blend: wgpu::BlendDescriptor {
                    src_factor: wgpu::BlendFactor::SrcAlpha,
                    dst_factor: wgpu::BlendFactor::OneMinusSrcAlpha,
                    operation: wgpu::BlendOperation::Add,
                },
                write_mask: wgpu::ColorWrite::ALL,
            }],
            depth_stencil_state: None,
            vertex_state: wgpu::VertexStateDescriptor {
                index_format: wgpu::IndexFormat::Uint16,
                vertex_buffers: &[wgpu::VertexBufferDescriptor {
                    stride: vertex_size as wgpu::BufferAddress,
                    step_mode: wgpu::InputStepMode::Vertex,
                    attributes: &[
                        // Position
                        wgpu::VertexAttributeDescriptor {
                            format: wgpu::VertexFormat::Float3,
                            offset: 0,
                            shader_location: 0,
                        },
                        // Color
                        wgpu::VertexAttributeDescriptor {
                            format: wgpu::VertexFormat::Float4,
                            offset: 3 * 4,
                            shader_location: 1,
                        },
                    ],
                }],
            },
            sample_count: SAMPLE_COUNT,
            sample_mask: !0,
            alpha_to_coverage_enabled: false,
        });

        //****************************** Setting up voxel pipeline ******************************
        let mc = mesh_count as u64;
        let vertex_buf_voxel = device.create_buffer(&wgpu::BufferDescriptor {
            label: None,
            // We can have a maximum number of mc * mc * mc voxel
            // Each voxel has 24 vertices
            size: mc * mc * mc * 24 * mem::size_of::<VoxelVertex>() as u64,
            usage: wgpu::BufferUsage::VERTEX | wgpu::BufferUsage::COPY_DST,
        });

        let index_buf_voxel = device.create_buffer(&wgpu::BufferDescriptor {
            label: None,
            // We can have a maximum number of mc * mc * mc voxel
            // We use 36 indices to draw a voxel
            size: mc * mc * mc * 36 * mem::size_of::<u32>() as u64,
            usage: wgpu::BufferUsage::INDEX | wgpu::BufferUsage::COPY_DST,
        });

        let light_uniform_buf = device.create_buffer(&wgpu::BufferDescriptor {
            label: None,
            size: mem::size_of::<LightRaw>() as u64,
            usage: wgpu::BufferUsage::UNIFORM | wgpu::BufferUsage::COPY_DST,
        });

        // Create pipeline layout
        let bind_group_layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: None,
            bindings: &[
                wgpu::BindGroupLayoutEntry {
                    binding: 0,
                    visibility: wgpu::ShaderStage::VERTEX,
                    ty: wgpu::BindingType::UniformBuffer { dynamic: false },
                },
                wgpu::BindGroupLayoutEntry {
                    binding: 1, // lights
                    visibility: wgpu::ShaderStage::VERTEX,
                    ty: wgpu::BindingType::UniformBuffer { dynamic: false },
                },
            ],
        });
        let pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            bind_group_layouts: &[&bind_group_layout],
        });

        // Create bind group
        let voxel_bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
            layout: &bind_group_layout,
            bindings: &[
                wgpu::Binding {
                    binding: 0,
                    resource: wgpu::BindingResource::Buffer {
                        buffer: &uniform_buf,
                        range: 0..uniform_buf_size,
                    },
                },
                wgpu::Binding {
                    binding: 1,
                    resource: wgpu::BindingResource::Buffer {
                        buffer: &light_uniform_buf,
                        range: 0..mem::size_of::<LightRaw>() as u64,
                    },
                },
            ],
            label: None,
        });

        let vs_bytes = include_bytes!("../shaders/voxel_new.vert.spv");
        let fs_bytes = include_bytes!("../shaders/voxel_new.frag.spv");

        // Create the voxel rendering pipeline
        let vs_module_voxel = device
            .create_shader_module(&wgpu::read_spirv(std::io::Cursor::new(&vs_bytes[..])).unwrap());
        let fs_module_voxel = device
            .create_shader_module(&wgpu::read_spirv(std::io::Cursor::new(&fs_bytes[..])).unwrap());

        let voxel_pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            layout: &pipeline_layout,
            vertex_stage: wgpu::ProgrammableStageDescriptor {
                module: &vs_module_voxel,
                entry_point: "main",
            },
            fragment_stage: Some(wgpu::ProgrammableStageDescriptor {
                module: &fs_module_voxel,
                entry_point: "main",
            }),
            rasterization_state: Some(wgpu::RasterizationStateDescriptor {
                front_face: wgpu::FrontFace::Ccw,
                cull_mode: wgpu::CullMode::Back,
                depth_bias: 0,
                depth_bias_slope_scale: 0.0,
                depth_bias_clamp: 0.0,
            }),
            primitive_topology: wgpu::PrimitiveTopology::TriangleList,
            color_states: &[wgpu::ColorStateDescriptor {
                format: sc_desc.format,
                color_blend: wgpu::BlendDescriptor::REPLACE,
                alpha_blend: wgpu::BlendDescriptor::REPLACE,
                write_mask: wgpu::ColorWrite::ALL,
            }],
            depth_stencil_state: Some(wgpu::DepthStencilStateDescriptor {
                format: DEPTH_FORMAT,
                depth_write_enabled: true,
                depth_compare: wgpu::CompareFunction::Less,
                stencil_front: wgpu::StencilStateFaceDescriptor::IGNORE,
                stencil_back: wgpu::StencilStateFaceDescriptor::IGNORE,
                stencil_read_mask: 0,
                stencil_write_mask: 0,
            }),
            vertex_state: wgpu::VertexStateDescriptor {
                index_format: wgpu::IndexFormat::Uint32,
                vertex_buffers: &[wgpu::VertexBufferDescriptor {
                    stride: mem::size_of::<VoxelVertex>() as wgpu::BufferAddress,
                    step_mode: wgpu::InputStepMode::Vertex,
                    attributes: &[
                        // Position
                        wgpu::VertexAttributeDescriptor {
                            format: wgpu::VertexFormat::Float3,
                            offset: 0,
                            shader_location: 0,
                        },
                        // Color
                        wgpu::VertexAttributeDescriptor {
                            format: wgpu::VertexFormat::Float4,
                            offset: 3 * 4,
                            shader_location: 1,
                        },
                        // Normal
                        wgpu::VertexAttributeDescriptor {
                            format: wgpu::VertexFormat::Float4,
                            offset: 7 * 4,
                            shader_location: 2,
                        },
                    ],
                }],
            },
            sample_count: SAMPLE_COUNT,
            sample_mask: !0,
            alpha_to_coverage_enabled: false,
        });

        let ui_pipeline = build_ui_pipeline(&device);
        let multisampled_framebuffer = create_texture_view(
            &device,
            &sc_desc,
            SAMPLE_COUNT,
            sc_desc.format,
            Some("MSAAFrameBuffer"),
        );

        let depth_buffer = create_texture_view(
            &device,
            &sc_desc,
            SAMPLE_COUNT,
            DEPTH_FORMAT,
            Some("DepthBuffer"),
        );

        Renderer {
            surface,
            device,
            queue,
            sc_desc,
            swap_chain,
            depth_buffer,
            camera,
            mesh_pipeline: Pipeline {
                pipeline: mesh_pipeline,
                bind_group: mesh_bind_group,
                vertex_buf: vertex_buf_mesh,
                index_buf: index_buf_mesh,
                index_count: mesh_index_data.len(),
            },
            cursor_pipeline: Pipeline {
                pipeline: cursor_pipeline,
                bind_group: cursor_bind_group,
                vertex_buf: vertex_buf_cursor,
                index_buf: index_buf_cursor,
                index_count: cursor_index_data.len(),
            },
            voxel_pipeline: Pipeline {
                pipeline: voxel_pipeline,
                bind_group: voxel_bind_group,
                vertex_buf: vertex_buf_voxel,
                index_buf: index_buf_voxel,
                index_count: 0,
            },
            cursor_cube,
            draw_cube: None,
            render_cursor: true,
            mvp_buf: uniform_buf,
            multisampled_framebuffer,
            voxel_manager: VoxelManager::new(mesh_count as usize),
            mesh_count,
            light: Light {
                pos: cgmath::Point3::new(DEFAULT_MESH_COUNT as f32 + 2.0, DEFAULT_MESH_COUNT as f32 + 2.0, DEFAULT_MESH_COUNT as f32 + 2.0),
                color: wgpu::Color {
                    r: 1.0,
                    g: 1.0,
                    b: 1.0,
                    a: 1.0,
                },
            },
            light_uniform_buf,
            lights_are_dirty: true,
            command_buffers: Vec::new(),
            ui_pipeline,
        }
    }

    pub fn get_model_data(&self) -> (Vec<VoxelVertex>, Vec<u32>) {
        self.voxel_manager.vertices()
    }

    pub fn update_view(&mut self, event: winit::event::WindowEvent) {
        let viewport_changed = self.camera.update(&event);
        if viewport_changed {
            let matrices = self
                .camera
                .mvp_matrices(self.sc_desc.width as f32 / self.sc_desc.height as f32);
            let matrices_ref = matrices.as_ref();
            Self::write_buffer(
                &self.device,
                bytemuck::cast_slice(matrices_ref),
                &self.mvp_buf,
                &mut self.command_buffers,
            );
        }
    }

    fn get_grid_pos(world_pos: cgmath::Vector3<f32>) -> cgmath::Vector3<f32> {
        cgmath::Vector3::new(world_pos.x.floor(), world_pos.y.ceil(), world_pos.z.ceil())
    }

    pub fn update_cursor_pos_on_plane(&mut self, pos: cgmath::Vector3<f32>, plane: Option<&Plane>) {
        if let Some(plane) = plane {
            self.cursor_cube = BoundingBox::new(
                Self::get_grid_pos(pos),
                plane.left + plane.down + plane.normal,
                HALF_ALPHA_RED.into(),
            );
            let vertex_data = self.cursor_cube.vertices();
            Self::write_buffer(
                &self.device,
                bytemuck::cast_slice(&vertex_data),
                &self.cursor_pipeline.vertex_buf,
                &mut self.command_buffers,
            );
            self.render_cursor = true;
        } else {
            self.render_cursor = false;
        }
    }

    pub fn update_cursor_pos(&mut self, bbox: BoundingBox) {
        self.cursor_cube = bbox;
        self.cursor_cube.color = HALF_ALPHA_RED.into();
        let vertex_data = self.cursor_cube.vertices();
        Self::write_buffer(
            &self.device,
            bytemuck::cast_slice(&vertex_data),
            &self.cursor_pipeline.vertex_buf,
            &mut self.command_buffers,
        );
        self.render_cursor = true;
    }

    pub fn update_draw_rectangle_on_plane(&mut self, pos: cgmath::Vector3<f32>, plane: Option<&Plane>) {
        if let Some(plane) = plane {
            let end_cube = BoundingBox::new(
                Self::get_grid_pos(pos),
                plane.left + plane.down + plane.normal,
                HALF_ALPHA_RED.into(),
            );
            let draw_cube = self.cursor_cube.containing_box(&end_cube);
            let vertex_data = draw_cube.vertices();
            Self::write_buffer(
                &self.device,
                bytemuck::cast_slice(&vertex_data),
                &self.cursor_pipeline.vertex_buf,
                &mut self.command_buffers,
            );
            self.draw_cube = Some(draw_cube);
            self.render_cursor = true;
        } else {
            self.render_cursor = false;
        }
    }

    pub fn update_draw_rectangle(&mut self, mut bbox: BoundingBox) {
        bbox.color = HALF_ALPHA_RED.into();
        let draw_cube = self.cursor_cube.containing_box(&bbox);
        let vertex_data = draw_cube.vertices();
        Self::write_buffer(
            &self.device,
            bytemuck::cast_slice(&vertex_data),
            &self.cursor_pipeline.vertex_buf,
            &mut self.command_buffers,
        );
        self.draw_cube = Some(draw_cube);
        self.render_cursor = true;
    }

    pub fn draw_rectangle(&mut self, color: [f32; 4]) {
        if let Some(mut cube) = self.draw_cube.take() {
            cube.rearrange();
            cube.color = color;
            self.voxel_manager.add_box(cube);
            let (vertex_data, index_data) = self.voxel_manager.vertices();
            Self::write_buffer(
                &self.device,
                bytemuck::cast_slice(&vertex_data),
                &self.voxel_pipeline.vertex_buf,
                &mut self.command_buffers,
            );
            Self::write_buffer(
                &self.device,
                bytemuck::cast_slice(&index_data),
                &self.voxel_pipeline.index_buf,
                &mut self.command_buffers,
            );
            self.voxel_pipeline.index_count = index_data.len();
        }
    }

    #[cfg(feature = "debug_ray")]
    pub fn debug_update(&mut self) {
        let (vertex_data, index_data) = self.voxel_manager.vertices();
        if vertex_data.len() == 0 {
            return
        }
        Self::write_buffer(
            &self.device,
            bytemuck::cast_slice(&vertex_data),
            &self.voxel_pipeline.vertex_buf,
            &mut self.command_buffers,
        );
        Self::write_buffer(
            &self.device,
            bytemuck::cast_slice(&index_data),
            &self.voxel_pipeline.index_buf,
            &mut self.command_buffers,
        );
    }

    pub fn erase_rectangle(&mut self) {
        if let Some(mut cube) = self.draw_cube.take() {
            cube.rearrange();
            self.voxel_manager.erase_box(cube);
            let (vertex_data, index_data) = self.voxel_manager.vertices();
            if index_data.len() > 0 {
                Self::write_buffer(
                    &self.device,
                    bytemuck::cast_slice(&vertex_data),
                    &self.voxel_pipeline.vertex_buf,
                    &mut self.command_buffers,
                );
                Self::write_buffer(
                    &self.device,
                    bytemuck::cast_slice(&index_data),
                    &self.voxel_pipeline.index_buf,
                    &mut self.command_buffers,
                );
            }
            self.voxel_pipeline.index_count = index_data.len();
        }
    }

    #[cfg(feature = "debug_ray")]
    pub fn cursor_helper(
        &mut self,
        near_pos: Option<cgmath::Vector3<f32>>,
        far_pos: cgmath::Vector3<f32>,
    ) {
        let (mut vertex_data, mut index_data) = generate_mesh_vertices(self.mesh_count);
        for _ in 0..2 {
            vertex_data.pop();
            index_data.pop();
        }
        vertex_data.push(vertex(
            near_pos
                .unwrap_or(cgmath::Vector3::new(0.5, 0.5, 0.5))
                .into(),
            RED,
        ));
        index_data.push((vertex_data.len() - 1) as u16);
        vertex_data.push(vertex(far_pos.into(), BLUE));
        index_data.push((vertex_data.len() - 1) as u16);
        Self::write_buffer(
            &self.device,
            bytemuck::cast_slice(&vertex_data),
            &self.mesh_pipeline.vertex_buf,
            &mut self.command_buffers,
        );
        Self::write_buffer(
            &self.device,
            bytemuck::cast_slice(&index_data),
            &self.mesh_pipeline.index_buf,
            &mut self.command_buffers,
        );
    }

    pub fn resize(&mut self, size: winit::dpi::PhysicalSize<u32>) {
        self.sc_desc.width = size.width;
        self.sc_desc.height = size.height;
        self.swap_chain = self.device.create_swap_chain(&self.surface, &self.sc_desc);
        let matrices = self
            .camera
            .mvp_matrices(self.sc_desc.width as f32 / self.sc_desc.height as f32);
        let matrices_ref = matrices.as_ref();
        Self::write_buffer(
            &self.device,
            bytemuck::cast_slice(matrices_ref),
            &self.mvp_buf,
            &mut self.command_buffers,
        );

        self.multisampled_framebuffer = create_texture_view(
            &self.device,
            &self.sc_desc,
            SAMPLE_COUNT,
            self.sc_desc.format,
            Some("MSAAFrameBuffer"),
        );
        self.depth_buffer = create_texture_view(
            &self.device,
            &self.sc_desc,
            SAMPLE_COUNT,
            DEPTH_FORMAT,
            Some("DepthBuffer"),
        );
    }

    pub fn write_buffer(
        device: &wgpu::Device,
        data: &[u8],
        buffer: &wgpu::Buffer,
        command_buffers: &mut Vec<wgpu::CommandBuffer>,
    ) {
        let mut encoder =
            device.create_command_encoder(&wgpu::CommandEncoderDescriptor { label: None });
        let temp_buf = device.create_buffer_with_data(data, wgpu::BufferUsage::COPY_SRC);

        encoder.copy_buffer_to_buffer(&temp_buf, 0, &buffer, 0, data.len() as u64);
        let command_buf = encoder.finish();
        command_buffers.push(command_buf);
    }

    pub fn render(&mut self, ui: &mut Ui) -> Interaction {
        #[cfg(feature = "debug_ray")]
        self.debug_update();

        let frame = match self.swap_chain.get_next_texture() {
            Ok(frame) => frame,
            Err(_) => {
                self.swap_chain = self.device.create_swap_chain(&self.surface, &self.sc_desc);
                self.swap_chain
                    .get_next_texture()
                    .expect("Failed to acquire next swap chain texture!")
            }
        };

        if self.lights_are_dirty {
            self.lights_are_dirty = false;
            Self::write_buffer(
                &self.device,
                bytemuck::bytes_of(&self.light.to_raw()),
                &self.light_uniform_buf,
                &mut self.command_buffers,
            );
        }

        let mut encoder = self
            .device
            .create_command_encoder(&wgpu::CommandEncoderDescriptor { label: None });
        {
            let mut rpass_depth = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                color_attachments: &[wgpu::RenderPassColorAttachmentDescriptor {
                    attachment: &self.multisampled_framebuffer,
                    resolve_target: Some(&frame.view),
                    load_op: wgpu::LoadOp::Clear,
                    store_op: wgpu::StoreOp::Store,
                    clear_color: wgpu::Color {
                        r: 0.0,
                        g: 0.8,
                        b: 1.0,
                        a: 1.0,
                    },
                }],
                depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachmentDescriptor {
                    attachment: &self.depth_buffer,
                    depth_load_op: wgpu::LoadOp::Clear,
                    depth_store_op: wgpu::StoreOp::Store,
                    stencil_load_op: wgpu::LoadOp::Clear,
                    stencil_store_op: wgpu::StoreOp::Store,
                    clear_depth: 1.0,
                    clear_stencil: 0,
                }),
            });
            self.mesh_pipeline.draw(&mut rpass_depth);
            if self.voxel_pipeline.index_count > 0 {
                self.voxel_pipeline.draw(&mut rpass_depth);
            }
        }
        {
            let mut rpass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                color_attachments: &[wgpu::RenderPassColorAttachmentDescriptor {
                    attachment: &self.multisampled_framebuffer,
                    resolve_target: Some(&frame.view),
                    load_op: wgpu::LoadOp::Load,
                    store_op: wgpu::StoreOp::Store,
                    clear_color: wgpu::Color::BLACK,
                }],
                depth_stencil_attachment: None,
            });
            if self.render_cursor {
                self.cursor_pipeline.draw(&mut rpass);
            }
        }
        // Render ui
        {
            let mut rpass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                color_attachments: &[wgpu::RenderPassColorAttachmentDescriptor {
                    attachment: &frame.view,
                    resolve_target: None,
                    load_op: wgpu::LoadOp::Load,
                    store_op: wgpu::StoreOp::Store,
                    clear_color: wgpu::Color::BLACK,
                }],
                depth_stencil_attachment: None,
            });
            rpass.set_pipeline(&self.ui_pipeline);
        }

        let mouse_interaction = ui.draw(&mut self.device, &mut encoder, &frame.view);

        let mut command_buffers = self.command_buffers.drain(..).collect::<Vec<_>>();
        command_buffers.push(encoder.finish());
        self.queue.submit(&command_buffers);
        mouse_interaction
    }
}
