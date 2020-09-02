use crate::geometry::{BoundingBox, Ray, ray_box_intersection};
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

    pub fn get_intersection_box(&mut self, ray: &Ray) -> (Option<BoundingBox>, Vector3<f32>) {
        let mut closest_intersect_box = None;
        let mut closest_fraction = 100000.0;
        let mut closest_intersection = Vector3::new(0.0, 0.0, 0.0);
        let mut intersection = Vector3::new(0.0, 0.0, 0.0);
        let mut fraction = 1.0;
        let mut bbox;
        for x in 0..self.extent {
            for y in 0..self.extent {
                for z in 0..self.extent {
                    if let Some(desc) = self.boxes[x][y][z] {
                        bbox = BoundingBox::new(
                            cgmath::Vector3::new(x as f32, y as f32, z as f32),
                            cgmath::Vector3::new(1.0, 1.0, 1.0),
                            desc.color,
                        );

                        if ray_box_intersection(&bbox, ray, &mut fraction, &mut intersection) {
                            if fraction.abs() < closest_fraction {
                                closest_fraction = fraction.abs();
                                closest_intersect_box = Some(bbox);
                                closest_intersection = intersection;
                                if cfg!(feature = "debug_ray") {
                                    self.boxes[x][y][z] = Some(CubeDescriptor::new([0.0, 0.0, 1.0, 1.0]));
                                }
                            }
                        } else if cfg!(feature = "debug_ray") {
                            self.boxes[x][y][z] = Some(CubeDescriptor::new([0.0, 1.0, 0.0, 0.1]));
                        }

                    }
                }
            }
        }
        return (closest_intersect_box, closest_intersection);
    }
}
