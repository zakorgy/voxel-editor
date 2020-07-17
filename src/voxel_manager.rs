use crate::geometry::Cuboid;
use cgmath::Vector3;

bitflags! {
    pub struct CubeFace: u32 {
        const FRONT =  0b00000001;
        const BACK =   0b00000010;
        const TOP =    0b00000100;
        const BOTTOM = 0b00001000;
        const LEFT =   0b00010000;
        const RIGHT =  0b00100000;
    }
}

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
}
