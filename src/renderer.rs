use crate::camera::CameraWrapper;
use crate::geometry::*;
use crate::voxel_manager::VoxelManager;
use cgmath;
use wgpu;

pub const DEFAULT_MESH_COUNT: u16 = 16;
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
    _pos: [f32; 4],
    _col: [f32; 4],
}

#[derive(Clone, Copy)]
pub struct VoxelVertex {
    _pos: [f32; 4],
    _col: [f32; 4],
    _normal: [f32; 3],
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
        _pos: [pos[0], pos[1], pos[2], 1.0],
        _col: col,
    }
}

fn voxel_vertex(pos: [f32; 3], col: [f32; 4], norm: [f32; 3]) -> VoxelVertex {
    VoxelVertex {
        _pos: [pos[0], pos[1], pos[2], 1.0],
        _col: col,
        _normal: norm,
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

impl Cuboid {
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
    pub fn vertices(&self) -> (Vec<VoxelVertex>, Vec<u16>) {
        let mut vertex_data = Vec::new();
        let mut index_data = Vec::new();
        let mut step;
        let mut idx;
        let mut cube;
        for x in 0..self.extent {
            for y in 0..self.extent {
                for z in 0..self.extent {
                    if let Some(desc) = self.cubes[x][y][z] {
                        let cube_faces = self.visible_faces(cgmath::Vector3::new(x, y, z));
                        println!("Visible faces {:?}", cube_faces);
                        idx = vertex_data.len() as u16;
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
                        cube = Cuboid::new(
                            cgmath::Vector3::new(x as f32, y as f32, z as f32),
                            cgmath::Vector3::new(1.0, 1.0, 1.0),
                            desc.color,
                        );
                        vertex_data.append(&mut cube.voxel_vertices());
                    }
                }
            }
        }
        (vertex_data, index_data)
    }
}

fn generate_cursor_vertices(cuboid: &Cuboid) -> (Vec<Vertex>, Vec<u16>) {
    let index_data: Vec<u16> = vec![
        0, 1, 2, 2, 3, 0, 4, 5, 6, 6, 7, 4, 8, 9, 10, 10, 11, 8, 12, 13, 14, 14, 15, 12, 16, 17,
        18, 18, 19, 16, 20, 21, 22, 22, 23, 20,
    ];
    (cuboid.vertices(), index_data)
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
        render_pass.set_index_buffer(self.index_buf.slice(..));
        render_pass.set_vertex_buffer(0, self.vertex_buf.slice(..));
        render_pass.draw_indexed(0..self.index_count as u32, 0, 0..1);
    }
}

pub struct Renderer {
    pub camera: CameraWrapper,
    surface: wgpu::Surface,
    device: wgpu::Device,
    queue: wgpu::Queue,
    sc_desc: wgpu::SwapChainDescriptor,
    swap_chain: wgpu::SwapChain,
    depth_buffer: wgpu::TextureView,
    mesh_pipeline: Pipeline,
    render_cursor: bool,
    cursor_pipeline: Pipeline,
    voxel_pipeline: Pipeline,
    cursor_cube: Cuboid,
    draw_cube: Option<Cuboid>,
    mvp_buf: wgpu::Buffer,
    multisampled_framebuffer: wgpu::TextureView,
    pub mesh_count: u16,
    voxel_manager: VoxelManager,
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
            bindings: &[wgpu::BindGroupLayoutEntry::new(
                0,
                wgpu::ShaderStage::VERTEX,
                wgpu::BindingType::UniformBuffer {
                    dynamic: false,
                    min_binding_size: wgpu::BufferSize::new(
                        mem::size_of::<cgmath::Matrix4<f32>>() as _
                    ),
                },
            )],
        });
        let pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            bind_group_layouts: &[&bind_group_layout],
        });

        let mut camera = CameraWrapper::new(
            sc_desc.width as f32 / sc_desc.height as f32,
            mesh_count as f32,
        );

        let mx = camera.mvp_matrix(sc_desc.width as f32 / sc_desc.height as f32);
        let mx_ref = mx.as_ref();
        let uniform_buf = device.create_buffer_with_data(
            bytemuck::cast_slice(mx_ref),
            wgpu::BufferUsage::UNIFORM | wgpu::BufferUsage::COPY_DST,
        );

        // Create bind group
        let mesh_bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
            layout: &bind_group_layout,
            bindings: &[wgpu::Binding {
                binding: 0,
                resource: wgpu::BindingResource::Buffer(uniform_buf.slice(..)),
            }],
            label: None,
        });

        // Create the mesh rendering pipeline
        let vs_module = device.create_shader_module(wgpu::include_spirv!("shader.vert.spv"));
        let fs_module = device.create_shader_module(wgpu::include_spirv!("shader.frag.spv"));

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
                            format: wgpu::VertexFormat::Float4,
                            offset: 0,
                            shader_location: 0,
                        },
                        // Color
                        wgpu::VertexAttributeDescriptor {
                            format: wgpu::VertexFormat::Float4,
                            offset: 4 * 4,
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
        let cursor_cube = Cuboid::new(
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
            bindings: &[wgpu::BindGroupLayoutEntry::new(
                0,
                wgpu::ShaderStage::VERTEX,
                wgpu::BindingType::UniformBuffer {
                    dynamic: false,
                    min_binding_size: wgpu::BufferSize::new(
                        mem::size_of::<cgmath::Matrix4<f32>>() as _
                    ),
                },
            )],
        });
        let pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            bind_group_layouts: &[&bind_group_layout],
        });

        // Create bind group
        let cursor_bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
            layout: &bind_group_layout,
            bindings: &[wgpu::Binding {
                binding: 0,
                resource: wgpu::BindingResource::Buffer(uniform_buf.slice(..)),
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
                            format: wgpu::VertexFormat::Float4,
                            offset: 0,
                            shader_location: 0,
                        },
                        // Color
                        wgpu::VertexAttributeDescriptor {
                            format: wgpu::VertexFormat::Float4,
                            offset: 4 * 4,
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
            mapped_at_creation: false,
        });

        let index_buf_voxel = device.create_buffer(&wgpu::BufferDescriptor {
            label: None,
            // We can have a maximum number of mc * mc * mc voxel
            // We use 36 indices to draw a voxel
            size: mc * mc * mc * 36 * mem::size_of::<u16>() as u64,
            usage: wgpu::BufferUsage::INDEX | wgpu::BufferUsage::COPY_DST,
            mapped_at_creation: false,
        });

        // Create pipeline layout
        let bind_group_layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: None,
            bindings: &[wgpu::BindGroupLayoutEntry::new(
                0,
                wgpu::ShaderStage::VERTEX,
                wgpu::BindingType::UniformBuffer {
                    dynamic: false,
                    min_binding_size: wgpu::BufferSize::new(
                        mem::size_of::<cgmath::Matrix4<f32>>() as _
                    ),
                },
            )],
        });
        let pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            bind_group_layouts: &[&bind_group_layout],
        });

        // Create bind group
        let voxel_bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
            layout: &bind_group_layout,
            bindings: &[wgpu::Binding {
                binding: 0,
                resource: wgpu::BindingResource::Buffer(uniform_buf.slice(..)),
            }],
            label: None,
        });

        // Create the voxel rendering pipeline
        let vs_module_voxel =
            device.create_shader_module(wgpu::include_spirv!("voxel_shader.vert.spv"));
        let fs_module_voxel =
            device.create_shader_module(wgpu::include_spirv!("voxel_shader.frag.spv"));

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
                index_format: wgpu::IndexFormat::Uint16,
                vertex_buffers: &[wgpu::VertexBufferDescriptor {
                    stride: mem::size_of::<VoxelVertex>() as wgpu::BufferAddress,
                    step_mode: wgpu::InputStepMode::Vertex,
                    attributes: &[
                        // Position
                        wgpu::VertexAttributeDescriptor {
                            format: wgpu::VertexFormat::Float4,
                            offset: 0,
                            shader_location: 0,
                        },
                        // Color
                        wgpu::VertexAttributeDescriptor {
                            format: wgpu::VertexFormat::Float4,
                            offset: 4 * 4,
                            shader_location: 1,
                        },
                        // Normal
                        wgpu::VertexAttributeDescriptor {
                            format: wgpu::VertexFormat::Float4,
                            offset: 4 * 4,
                            shader_location: 2,
                        },
                    ],
                }],
            },
            sample_count: SAMPLE_COUNT,
            sample_mask: !0,
            alpha_to_coverage_enabled: false,
        });
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
        }
    }

    pub fn update_view(&mut self, event: winit::event::WindowEvent) {
        let viewport_changed = self.camera.update(&event);
        if viewport_changed {
            let mx = self
                .camera
                .mvp_matrix(self.sc_desc.width as f32 / self.sc_desc.height as f32);
            let mx_ref = mx.as_ref();
            self.queue
                .write_buffer(&self.mvp_buf, 0, bytemuck::cast_slice(mx_ref));
        }
    }

    fn get_grid_pos(world_pos: cgmath::Vector3<f32>) -> cgmath::Vector3<f32> {
        cgmath::Vector3::new(world_pos.x.floor(), world_pos.y.ceil(), world_pos.z.ceil())
    }

    pub fn update_cursor_pos(&mut self, pos: cgmath::Vector3<f32>, plane: Option<&Plane>) {
        if let Some(plane) = plane {
            self.cursor_cube = Cuboid::new(
                Self::get_grid_pos(pos),
                plane.left + plane.down + plane.normal,
                HALF_ALPHA_RED.into(),
            );
            let vertex_data = self.cursor_cube.vertices();
            self.queue.write_buffer(
                &self.cursor_pipeline.vertex_buf,
                0,
                bytemuck::cast_slice(&vertex_data),
            );
            self.render_cursor = true;
        } else {
            self.render_cursor = false;
        }
    }

    pub fn update_draw_rectangle(&mut self, pos: cgmath::Vector3<f32>, plane: Option<&Plane>) {
        if let Some(plane) = plane {
            let end_cube = Cuboid::new(
                Self::get_grid_pos(pos),
                plane.left + plane.down + plane.normal,
                HALF_ALPHA_RED.into(),
            );
            let draw_cube = self.cursor_cube.containing_cube(&end_cube);
            let vertex_data = draw_cube.vertices();
            self.queue.write_buffer(
                &self.cursor_pipeline.vertex_buf,
                0,
                bytemuck::cast_slice(&vertex_data),
            );
            self.draw_cube = Some(draw_cube);
            self.render_cursor = true;
        } else {
            self.render_cursor = false;
        }
    }

    pub fn draw_rectangle(&mut self) {
        if let Some(mut cube) = self.draw_cube.take() {
            cube.rearrange();
            cube.color = WHITE;
            self.voxel_manager.add_cube(cube);
            let (vertex_data, index_data) = self.voxel_manager.vertices();
            self.queue.write_buffer(
                &self.voxel_pipeline.vertex_buf,
                0,
                bytemuck::cast_slice(&vertex_data),
            );
            self.queue.write_buffer(
                &self.voxel_pipeline.index_buf,
                0,
                bytemuck::cast_slice(&index_data),
            );
            self.voxel_pipeline.index_count = index_data.len();
        }
    }

    pub fn erase_rectangle(&mut self) {
        if let Some(mut cube) = self.draw_cube.take() {
            cube.rearrange();
            self.voxel_manager.erase_cube(cube);
            let (vertex_data, index_data) = self.voxel_manager.vertices();
            self.queue.write_buffer(
                &self.voxel_pipeline.vertex_buf,
                0,
                bytemuck::cast_slice(&vertex_data),
            );
            self.queue.write_buffer(
                &self.voxel_pipeline.index_buf,
                0,
                bytemuck::cast_slice(&index_data),
            );
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
        self.queue.write_buffer(
            &self.mesh_pipeline.vertex_buf,
            0,
            bytemuck::cast_slice(&vertex_data),
        );
        self.queue.write_buffer(
            &self.mesh_pipeline.index_buf,
            0,
            bytemuck::cast_slice(&index_data),
        );
    }

    pub fn resize(&mut self, size: winit::dpi::PhysicalSize<u32>) {
        self.sc_desc.width = size.width;
        self.sc_desc.height = size.height;
        self.swap_chain = self.device.create_swap_chain(&self.surface, &self.sc_desc);
        let mx = self
            .camera
            .mvp_matrix(self.sc_desc.width as f32 / self.sc_desc.height as f32);
        let mx_ref = mx.as_ref();
        self.queue
            .write_buffer(&self.mvp_buf, 0, bytemuck::cast_slice(mx_ref));
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

    pub fn render(&mut self) {
        let frame = match self.swap_chain.get_next_frame() {
            Ok(frame) => frame,
            Err(_) => {
                self.swap_chain = self.device.create_swap_chain(&self.surface, &self.sc_desc);
                self.swap_chain
                    .get_next_frame()
                    .expect("Failed to acquire next swap chain texture!")
            }
        };

        let mut encoder = self
            .device
            .create_command_encoder(&wgpu::CommandEncoderDescriptor { label: None });
        {
            let mut rpass_depth = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                color_attachments: &[wgpu::RenderPassColorAttachmentDescriptor {
                    attachment: &self.multisampled_framebuffer,
                    resolve_target: Some(&frame.output.view),
                    ops: wgpu::Operations {
                        load: wgpu::LoadOp::Clear(wgpu::Color {
                            r: 0.0,
                            g: 0.8,
                            b: 1.0,
                            a: 1.0,
                        }),
                        store: true,
                    },
                }],
                depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachmentDescriptor {
                    attachment: &self.depth_buffer,
                    depth_ops: Some(wgpu::Operations {
                        load: wgpu::LoadOp::Clear(1.0),
                        store: true,
                    }),
                    stencil_ops: None,
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
                    resolve_target: Some(&frame.output.view),
                    ops: wgpu::Operations {
                        load: wgpu::LoadOp::Load,
                        store: true,
                    },
                }],
                depth_stencil_attachment: None,
            });
            if self.render_cursor {
                self.cursor_pipeline.draw(&mut rpass);
            }
        }

        let command_buf = encoder.finish();
        self.queue.submit(Some(command_buf));
    }
}
