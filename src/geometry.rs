use cgmath::{InnerSpace, Matrix4, Transform, Vector3, Vector4};

pub const EPSYLON: f32 = 0.000001;

pub struct Plane {
    pub point: Vector3<f32>,
    pub normal: Vector3<f32>,
    pub left: Vector3<f32>,
    pub down: Vector3<f32>,
    #[cfg(feature = "debug_ray")]
    pub name: &'static str,
}

pub const XY_PLANE: Plane = Plane {
    point: Vector3 {x: 0.5, y: 0.5, z: 0.0},
    normal: Vector3 { x: 0.0, y: 0.0, z: 1.0 },
    left: Vector3 { x: 1.0, y: 0.0, z: 0.0 },
    down: Vector3 { x: 0.0, y: -1.0, z: 0.0 },
    #[cfg(feature = "debug_ray")]
    name: "XY",
};

pub const YZ_PLANE: Plane = Plane {
    point: Vector3 {x: 0.0, y: 0.5, z: 0.5},
    normal: Vector3 { x: 1.0, y: 0.0, z: 0.0 },
    left: Vector3 { x: 0.0, y: 0.0, z: -1.0 },
    down: Vector3 { x: 0.0, y: -1.0, z: 0.0 },
    #[cfg(feature = "debug_ray")]
    name: "YZ",
};

pub const XZ_PLANE: Plane = Plane {
    point: Vector3 {x: 0.5, y: 0.0, z: 0.5},
    normal: Vector3 { x: 0.0, y: 1.0, z: 0.0 },
    left: Vector3 { x: 0.0, y: 0.0, z: -1.0 },
    down: Vector3 { x: 1.0, y: 0.0, z: 0.0 },
    #[cfg(feature = "debug_ray")]
    name: "XZ",
};

pub struct Ray {
    pub point: Vector3<f32>,
    pub vector: Vector3<f32>,
}

impl Ray {
    pub fn new(
        point: Vector3<f32>,
        vector: Vector3<f32>,
    ) -> Self {
        Ray {
            point,
            vector,
        }
    }

    pub fn plane_intersection(
        &self,
        plane: &Plane,
    ) -> Option<Vector3<f32>> {
        let dist_square = self.vector.dot(plane.normal);
        if dist_square.abs() > EPSYLON {
            let diff = self.point - plane.point;
            let dist_square2 = diff.dot(plane.normal) / dist_square;
            if dist_square2 >= EPSYLON {
                return Some(self.point - self.vector * dist_square2)
            }
        }
        None
    }
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
        2.0 * winz - 1.0,
        1.0,
    );

    let mut out = matrix * in_vec;
    out.w = 1.0 / out.w;

    Vector3::new(out.x * out.w, out.y * out.w, out.z * out.w)
}
