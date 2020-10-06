use crate::camera::CameraWrapper;
use crate::vertex::*;
use cgmath::{InnerSpace, Matrix4, Transform, Vector3, Vector4};

pub const EPSYLON: f32 = 0.000001;

pub struct Plane {
    point: Vector3<f32>,
    pub normal: Vector3<f32>,
    pub left: Vector3<f32>,
    pub down: Vector3<f32>,
    #[cfg(feature = "debug_ray")]
    pub name: &'static str,
}

pub const XY_PLANE: Plane = Plane {
    point: Vector3 {
        x: 0.5,
        y: 0.5,
        z: 0.0,
    },
    normal: Vector3 {
        x: 0.0,
        y: 0.0,
        z: 1.0,
    },
    left: Vector3 {
        x: 1.0,
        y: 0.0,
        z: 0.0,
    },
    down: Vector3 {
        x: 0.0,
        y: -1.0,
        z: 0.0,
    },
    #[cfg(feature = "debug_ray")]
    name: "XY",
};

pub const YZ_PLANE: Plane = Plane {
    point: Vector3 {
        x: 0.0,
        y: 0.5,
        z: 0.5,
    },
    normal: Vector3 {
        x: 1.0,
        y: 0.0,
        z: 0.0,
    },
    left: Vector3 {
        x: 0.0,
        y: 0.0,
        z: -1.0,
    },
    down: Vector3 {
        x: 0.0,
        y: -1.0,
        z: 0.0,
    },
    #[cfg(feature = "debug_ray")]
    name: "YZ",
};

pub const XZ_PLANE: Plane = Plane {
    point: Vector3 {
        x: 0.5,
        y: 0.0,
        z: 0.5,
    },
    normal: Vector3 {
        x: 0.0,
        y: 1.0,
        z: 0.0,
    },
    left: Vector3 {
        x: 0.0,
        y: 0.0,
        z: -1.0,
    },
    down: Vector3 {
        x: 1.0,
        y: 0.0,
        z: 0.0,
    },
    #[cfg(feature = "debug_ray")]
    name: "XZ",
};

#[derive(Debug)]
pub struct Ray {
    pub origin: Vector3<f32>,
    pub end: Vector3<f32>,
    vector: Vector3<f32>,
}

impl Ray {
    pub fn new(origin: Vector3<f32>, end: Vector3<f32>) -> Self {
        Ray {
            origin,
            vector: origin - end,
            end,
        }
    }

    pub fn plane_intersection(&self, plane: &Plane) -> Option<Vector3<f32>> {
        let dist_square = self.vector.dot(plane.normal);
        if dist_square.abs() > EPSYLON {
            let diff = self.origin - plane.point;
            let dist_square2 = diff.dot(plane.normal) / dist_square;
            if dist_square2 >= EPSYLON {
                return Some(self.origin - self.vector * dist_square2);
            }
        }
        None
    }

    pub fn unproject(
        winx: f32,
        winy: f32,
        winz: f32,
        model_view: Matrix4<f32>,
        projection: Matrix4<f32>,
        window_size: winit::dpi::PhysicalSize<u32>,
    ) -> Vector3<f32> {
        let matrix = (projection * model_view).inverse_transform().unwrap();
        let in_vec = Vector4::new(
            (winx / window_size.width as f32) * 2.0 - 1.0,
            ((window_size.height as f32 - winy) / window_size.height as f32) * 2.0 - 1.0,
            winz,
            1.0,
        );
        let mut out = matrix * in_vec;
        out.w = 1.0 / out.w;
        Vector3::new(out.x * out.w, out.y * out.w, out.z * out.w)
    }

    pub fn from_cursor(
        &mut self,
        posx: f32,
        posy: f32,
        camera: &CameraWrapper,
        window_size: winit::dpi::PhysicalSize<u32>,
    ) {
        let mv_matrix = camera.model_view_mat();
        let proj_matrix = camera.projection_mat();

        let origin = Self::unproject(
            posx as f32,
            posy as f32,
            0.1 as f32,
            mv_matrix,
            proj_matrix,
            window_size,
        );

        let end = Self::unproject(
            posx as f32,
            posy as f32,
            1.0 as f32,
            mv_matrix,
            proj_matrix,
            window_size,
        );
        *self = Ray::new(origin, end);
    }
}

#[derive(Debug, Copy, Clone)]
pub struct BoundingBox {
    pub corner: Vector3<f32>,
    pub extent: Vector3<f32>,
    pub color: [f32; 4],
}

impl BoundingBox {
    fn vec_max(&self) -> Vector3<f32> {
        self.corner + self.extent
    }

    pub fn corner_points(&self) -> [Vector3<f32>; 8] {
        [
            self.corner,
            self.corner + Vector3::new(self.extent.x, 0.0, 0.0),
            self.corner + Vector3::new(self.extent.x, self.extent.y, 0.0),
            self.corner + Vector3::new(0.0, self.extent.y, 0.0),
            self.corner + Vector3::new(0.0, 0.0, self.extent.z),
            self.corner + Vector3::new(self.extent.x, 0.0, self.extent.z),
            self.corner + Vector3::new(self.extent.x, self.extent.y, self.extent.z),
            self.corner + Vector3::new(0.0, self.extent.y, self.extent.z),
        ]
    }

    fn manhattan_distance(start: &Vector3<f32>, end: &Vector3<f32>) -> f32 {
        (start.x - end.x).abs() + (start.y - end.y).abs() + (start.z - end.z).abs()
    }

    fn outermost_points(&self, other: &Self) -> (Vector3<f32>, Vector3<f32>) {
        let mut distance = 0.0;
        let mut outermost_points = (Vector3::unit_x(), Vector3::unit_x());
        for x in self.corner_points().iter() {
            for y in other.corner_points().iter() {
                let manhattan_dist = Self::manhattan_distance(x, y);
                if manhattan_dist > distance {
                    outermost_points = (*x, *y);
                    distance = manhattan_dist;
                }
            }
        }
        outermost_points
    }

    fn from_corner_points(origin: Vector3<f32>, end: Vector3<f32>, color: [f32; 4]) -> Self {
        BoundingBox {
            corner: origin,
            extent: end - origin,
            color,
        }
    }

    pub fn containing_box(&self, other: &Self) -> Self {
        let (origin, end) = self.outermost_points(other);
        Self::from_corner_points(origin, end, self.color)
    }

    pub fn new(corner: Vector3<f32>, extent: Vector3<f32>, color: [f32; 4]) -> Self {
        BoundingBox {
            corner,
            extent,
            color,
        }
    }

    pub fn rearrange(&mut self) {
        let corner_points = self.corner_points();
        let mut closest_to_origo = corner_points[0];
        let mut farthest_from_origo = corner_points[0];
        for point in corner_points[1..].iter() {
            if point.x <= closest_to_origo.x
                && point.y <= closest_to_origo.y
                && point.z <= closest_to_origo.z
            {
                closest_to_origo = *point;
            }

            if point.x >= farthest_from_origo.x
                && point.y >= farthest_from_origo.y
                && point.z >= farthest_from_origo.z
            {
                farthest_from_origo = *point;
            }
        }

        if self.corner == closest_to_origo {
            return;
        }
        *self = Self::from_corner_points(closest_to_origo, farthest_from_origo, self.color);
    }

    pub fn vertices(&self) -> Vec<Vertex> {
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

    pub fn voxel_vertices(&self) -> Vec<VoxelVertex> {
        let mut vertex_data = Vec::new();
        let corner_points = self.corner_points();
        // back
        /*0*/
        vertex_data.push(voxel_vertex(corner_points[0].into(), [0.0, 0.0, -1.0]));
        /*1*/
        vertex_data.push(voxel_vertex(corner_points[3].into(), [0.0, 0.0, -1.0]));
        /*2*/
        vertex_data.push(voxel_vertex(corner_points[2].into(), [0.0, 0.0, -1.0]));
        /*3*/
        vertex_data.push(voxel_vertex(corner_points[1].into(), [0.0, 0.0, -1.0]));

        // bottom
        /*4*/
        vertex_data.push(voxel_vertex(corner_points[1].into(), [0.0, -1.0, 0.0]));
        /*5*/
        vertex_data.push(voxel_vertex(corner_points[5].into(), [0.0, -1.0, 0.0]));
        /*6*/
        vertex_data.push(voxel_vertex(corner_points[4].into(), [0.0, -1.0, 0.0]));
        /*7*/
        vertex_data.push(voxel_vertex(corner_points[0].into(), [0.0, -1.0, 0.0]));

        // right
        /*9*/
        vertex_data.push(voxel_vertex(corner_points[2].into(), [1.0, 0.0, 0.0]));
        /*8*/
        vertex_data.push(voxel_vertex(corner_points[6].into(), [1.0, 0.0, 0.0]));
        /*10*/
        vertex_data.push(voxel_vertex(corner_points[5].into(), [1.0, 0.0, 0.0]));
        /*11*/
        vertex_data.push(voxel_vertex(corner_points[1].into(), [1.0, 0.0, 0.0]));

        // top
        /*12*/
        vertex_data.push(voxel_vertex(corner_points[3].into(), [0.0, 1.0, 0.0]));
        /*13*/
        vertex_data.push(voxel_vertex(corner_points[7].into(), [0.0, 1.0, 0.0]));
        /*14*/
        vertex_data.push(voxel_vertex(corner_points[6].into(), [0.0, 1.0, 0.0]));
        /*15*/
        vertex_data.push(voxel_vertex(corner_points[2].into(), [0.0, 1.0, 0.0]));

        // left
        /*16*/
        vertex_data.push(voxel_vertex(corner_points[3].into(), [-1.0, 0.0, 0.0]));
        /*17*/
        vertex_data.push(voxel_vertex(corner_points[0].into(), [-1.0, 0.0, 0.0]));
        /*18*/
        vertex_data.push(voxel_vertex(corner_points[4].into(), [-1.0, 0.0, 0.0]));
        /*19*/
        vertex_data.push(voxel_vertex(corner_points[7].into(), [-1.0, 0.0, 0.0]));

        // front
        /*20*/
        vertex_data.push(voxel_vertex(corner_points[4].into(), [0.0, 0.0, 1.0]));
        /*21*/
        vertex_data.push(voxel_vertex(corner_points[5].into(), [0.0, 0.0, 1.0]));
        /*22*/
        vertex_data.push(voxel_vertex(corner_points[6].into(), [0.0, 0.0, 1.0]));
        /*23*/
        vertex_data.push(voxel_vertex(corner_points[7].into(), [0.0, 0.0, 1.0]));

        vertex_data
    }
}

pub fn ray_box_intersection(bbox: &BoundingBox, ray: &Ray, dist: &mut f32) -> bool {
    let mut tmin = f32::NEG_INFINITY;
    let mut tmax = f32::INFINITY;

    for i in 0..3 {
        let t1 = (bbox.corner[i] - ray.origin[i]) / (ray.end[i] - ray.origin[i]);
        let t2 = (bbox.vec_max()[i] - ray.origin[i]) / (ray.end[i] - ray.origin[i]);

        tmin = tmin.max(t1.min(t2));
        tmax = tmax.min(t1.max(t2));
    }

    *dist = BoundingBox::manhattan_distance(&ray.origin, &bbox.corner);
    tmax >= tmin.max(0.0)
}

#[cfg(test)]
mod tests {

    use super::*;

    #[test]
    fn ray_box_intersection() {

        let black      = [0.0, 0.0, 0.0, 1.0];
        let origin     = Vector3::new(0.0, 0.0, 0.0);
        let ray_dir    = Vector3::new(1.0, 0.0, 0.0);
        let box_extent = Vector3::new(1.0, 1.0, 1.0);
        let box_corner = Vector3::new(0.0, -0.5, -0.5);

        let ray = Ray::new(origin, ray_dir);
        let bb  = BoundingBox::new(box_corner,
                                   box_extent,
                                   black);

        let mut dist = 0.0;
        assert!( super::ray_box_intersection(&bb, &ray, &mut dist) );
        assert_eq!( dist, 1.0 );
    }
}
