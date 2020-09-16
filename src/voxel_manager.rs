use crate::geometry::{ray_box_intersection, BoundingBox, Ray};
use crate::vertex::{VoxelInstance, VoxelVertex, instance};
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

    pub fn refill(&mut self, bbox: BoundingBox) {
        let origin: Vector3<usize> = Vector3::new(
            bbox.corner.x as usize,
            bbox.corner.y as usize,
            bbox.corner.z as usize,
        );
        for x in origin.x..origin.x + bbox.extent.x as usize {
            for y in origin.y..origin.y + bbox.extent.y as usize {
                for z in origin.z..origin.z + bbox.extent.z as usize {
                    if self.boxes[x][y][z].is_some()
                    {
                        self.boxes[x][y][z] = Some(CubeDescriptor::new(bbox.color.into()));
                    }
                }
            }
        }
    }

    fn get_neighbour_boxes(
        &mut self,
        pos_x: usize,
        pos_y: usize,
        pos_z: usize,
    ) -> Vec<BoundingBox> {
        let mut origins = Vec::new();
        let min_x = pos_x.max(1) - 1;
        let min_y = pos_y.max(1) - 1;
        let min_z = pos_z.max(1) - 1;

        let max_x = (pos_x + 1).min(self.extent - 1);
        let max_y = (pos_y + 1).min(self.extent - 1);
        let max_z = (pos_z + 1).min(self.extent - 1);
        for x in min_x..=max_x {
            if self.boxes[x][pos_y][pos_z].is_none() {
                origins.push(BoundingBox::new(
                    cgmath::Vector3::new(x as f32, pos_y as f32, pos_z as f32),
                    cgmath::Vector3::new(1.0, 1.0, 1.0),
                    [1.0; 4],
                ));
            }
        }
        for y in min_y..=max_y {
            if self.boxes[pos_x][y][pos_z].is_none() {
                origins.push(BoundingBox::new(
                    cgmath::Vector3::new(pos_x as f32, y as f32, pos_z as f32),
                    cgmath::Vector3::new(1.0, 1.0, 1.0),
                    [1.0; 4],
                ));
            }
        }
        for z in min_z..=max_z {
            if self.boxes[pos_x][pos_y][z].is_none() {
                origins.push(BoundingBox::new(
                    cgmath::Vector3::new(pos_x as f32, pos_y as f32, z as f32),
                    cgmath::Vector3::new(1.0, 1.0, 1.0),
                    [1.0; 4],
                ));
            }
        }
        origins
    }

    pub fn get_intersection_box(
        &mut self,
        ray: &Ray,
    ) -> (Option<BoundingBox>, Option<BoundingBox>) {
        let mut erase_box = None;
        let mut draw_box = None;
        let mut closest_distance = 100000.0;
        let mut distance = 1.0;
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

                        if ray_box_intersection(&bbox, ray, &mut distance) {
                            if distance.abs() < closest_distance {
                                closest_distance = distance.abs();
                                erase_box = Some(bbox);
                                if cfg!(feature = "debug_ray") {
                                    self.boxes[x][y][z] =
                                        Some(CubeDescriptor::new([0.0, 0.0, 1.0, 1.0]));
                                }
                            }
                        } else if cfg!(feature = "debug_ray") {
                            self.boxes[x][y][z] = Some(CubeDescriptor::new([0.0, 1.0, 0.0, 0.1]));
                        }
                    }
                }
            }
        }
        if let Some(bbox) = erase_box {
            draw_box = erase_box;
            let boxes = self.get_neighbour_boxes(
                bbox.corner.x as usize,
                bbox.corner.y as usize,
                bbox.corner.z as usize,
            );
            for bbox in boxes {
                if ray_box_intersection(&bbox, ray, &mut distance) {
                    if distance.abs() < closest_distance {
                        closest_distance = distance.abs();
                        draw_box = Some(bbox);
                    }
                }
            }
        }
        (erase_box, draw_box)
    }

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

    pub fn instance_data(&self) -> Vec<VoxelInstance> {
        let mut instance_data = Vec::new();
        for x in 0..self.extent {
            for y in 0..self.extent {
                for z in 0..self.extent {
                    if let Some(desc) = self.boxes[x][y][z] {
                        instance_data.push(instance([x as f32, y as f32, z as f32], desc.color))
                    }
                }
            }
        }
        instance_data
    }
}
