use bytemuck::{Pod, Zeroable};

const RED: [f32; 4] = [1.0, 0.0, 0.0, 1.0];
pub const HALF_ALPHA_RED: [f32; 4] = [1.0, 0.0, 0.0, 0.2];
const GREEN: [f32; 4] = [0.0, 1.0, 0.0, 1.0];
const BLUE: [f32; 4] = [0.0, 0.0, 1.0, 1.0];
const TRANSPARENT: [f32; 4] = [0.0, 0.0, 0.0, 0.0];

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

pub fn white_vertex(pos: [f32; 3]) -> Vertex {
    vertex(pos, [1.0; 4])
}

pub fn vertex(pos: [f32; 3], col: [f32; 4]) -> Vertex {
    Vertex {
        _pos: [pos[0], pos[1], pos[2], 1.0],
        _col: col,
    }
}

pub fn voxel_vertex(pos: [f32; 3], col: [f32; 4], norm: [f32; 3]) -> VoxelVertex {
    VoxelVertex {
        _pos: [pos[0], pos[1], pos[2], 1.0],
        _col: col,
        _normal: norm,
    }
}

pub fn generate_mesh_vertices(meshes: u16) -> (Vec<Vertex>, Vec<u16>) {
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
