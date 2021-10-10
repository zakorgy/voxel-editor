use crate::color::*;
use bytemuck::{Pod, Zeroable};

#[derive(Clone, Copy)]
pub struct Vertex {
    _pos: [f32; 3],
    _col: [f32; 4],
}

#[derive(Clone, Copy)]
pub struct VoxelVertex {
    pub pos: [f32; 3],
    pub normal: [f32; 3],
}

#[derive(Clone, Copy)]
pub struct VoxelInstance {
    _offset: [f32; 3],
    _col: [f32; 4],
}

unsafe impl Pod for Vertex {}
unsafe impl Zeroable for Vertex {}

unsafe impl Pod for VoxelInstance {}
unsafe impl Zeroable for VoxelInstance {}

unsafe impl Pod for VoxelVertex {}
unsafe impl Zeroable for VoxelVertex {}

pub fn white_vertex(pos: [f32; 3]) -> Vertex {
    vertex(pos, [1.0; 4])
}

pub fn vertex(pos: [f32; 3], col: [f32; 4]) -> Vertex {
    Vertex {
        _pos: [pos[0], pos[1], pos[2]],
        _col: col,
    }
}

pub fn instance(offset: [f32; 3], col: [f32; 4]) -> VoxelInstance {
    VoxelInstance {
        _offset: [offset[0], offset[1], offset[2]],
        _col: col,
    }
}

pub fn voxel_vertex(pos: [f32; 3], normal: [f32; 3]) -> VoxelVertex {
    VoxelVertex {
        pos: [pos[0], pos[1], pos[2]],
        normal,
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

#[cfg(test)]
mod tests {

    use super::*;
    #[test]
    fn test_mesh_vertices_1() {
        let mesh_size = 16;
        let (vertices, indices) = generate_mesh_vertices(mesh_size);
        assert_eq!(vertices.len(), (mesh_size * 12 + 6) as usize);
        assert_eq!(vertices.len(), indices.len());
    }

    #[test]
    fn test_mesh_vertices_2() {
        let mesh_size = 32;
        let (vertices, indices) = generate_mesh_vertices(mesh_size);
        assert_eq!(vertices.len(), (mesh_size * 12 + 6) as usize);
        assert_eq!(vertices.len(), indices.len());
    }

    #[test]
    fn test_mesh_vertices_3() {
        let mesh_size = 128;
        let (vertices, indices) = generate_mesh_vertices(mesh_size);
        assert_eq!(vertices.len(), (mesh_size * 12 + 6) as usize);
        assert_eq!(vertices.len(), indices.len());
    }
}
