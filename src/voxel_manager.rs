use crate::geometry::Cuboid;
use cgmath::Vector3;
use crate::vertex::VoxelVertex;

#[derive(Copy, Clone)]
pub struct CubeDescriptor {
    pub color: [f32; 4],
}

impl CubeDescriptor {
    fn new(color: [f32; 4]) -> Self {
        CubeDescriptor { color }
    }
}

pub struct VoxelManager {
    pub cubes: Vec<Vec<Vec<Option<CubeDescriptor>>>>,
    pub extent: usize,
}

impl VoxelManager {
    pub fn new(extent: usize) -> Self {
        VoxelManager {
            cubes: vec![vec![vec![None; extent]; extent]; extent],
            extent,
        }
    }

    pub fn add_cube(&mut self, cuboid: Cuboid) {
        let origin: Vector3<usize> = Vector3::new(
            cuboid.corner.x as usize,
            cuboid.corner.y as usize,
            cuboid.corner.z as usize,
        );
        for x in origin.x..origin.x + cuboid.extent.x as usize {
            for y in origin.y..origin.y + cuboid.extent.y as usize {
                for z in origin.z..origin.z + cuboid.extent.z as usize {
                    self.cubes[x][y][z] = Some(CubeDescriptor::new(cuboid.color.into()));
                }
            }
        }
    }

    pub fn erase_cube(&mut self, cuboid: Cuboid) {
        let origin: Vector3<usize> = Vector3::new(
            cuboid.corner.x as usize,
            cuboid.corner.y as usize,
            cuboid.corner.z as usize,
        );
        for x in origin.x..origin.x + cuboid.extent.x as usize {
            for y in origin.y..origin.y + cuboid.extent.y as usize {
                for z in origin.z..origin.z + cuboid.extent.z as usize {
                    self.cubes[x][y][z] = None;
                }
            }
        }
    }

    pub fn vertices(&self) -> (Vec<VoxelVertex>, Vec<u32>) {
        let mut vertex_data = Vec::new();
        let mut index_data = Vec::new();
        let mut step;
        let mut idx;
        let mut cube;
        for x in 0..self.extent {
            for y in 0..self.extent {
                for z in 0..self.extent {
                    if let Some(desc) = self.cubes[x][y][z] {
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
