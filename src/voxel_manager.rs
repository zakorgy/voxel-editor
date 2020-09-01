use crate::geometry::BoundingBox;
use cgmath::Vector3;

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
    pub boxes: Vec<Vec<Vec<Option<CubeDescriptor>>>>,
    pub extent: usize,
}

impl VoxelManager {
    pub fn new(extent: usize) -> Self {
        VoxelManager {
            boxes: vec![vec![vec![None; extent]; extent]; extent],
            extent,
        }
    }

    pub fn add_box(&mut self, bbox: BoundingBox) {
        let origin: Vector3<usize> = Vector3::new(
            bbox.corner.x as usize,
            bbox.corner.y as usize,
            bbox.corner.z as usize,
        );
        for x in origin.x..origin.x + bbox.extent.x as usize {
            for y in origin.y..origin.y + bbox.extent.y as usize {
                for z in origin.z..origin.z + bbox.extent.z as usize {
                    self.boxes[x][y][z] = Some(CubeDescriptor::new(bbox.color.into()));
                }
            }
        }
    }

    pub fn erase_box(&mut self, bbox: BoundingBox) {
        let origin: Vector3<usize> = Vector3::new(
            bbox.corner.x as usize,
            bbox.corner.y as usize,
            bbox.corner.z as usize,
        );
        for x in origin.x..origin.x + bbox.extent.x as usize {
            for y in origin.y..origin.y + bbox.extent.y as usize {
                for z in origin.z..origin.z + bbox.extent.z as usize {
                    self.boxes[x][y][z] = None;
                }
            }
        }
    }
}
