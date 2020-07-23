use bytemuck::{Pod, Zeroable};
use cgmath;

pub struct Light {
    pub pos: cgmath::Point3<f32>,
    pub color: wgpu::Color,
    pub fov: f32,
    pub depth: std::ops::Range<f32>,
}

#[repr(C)]
#[derive(Clone, Copy)]
pub struct LightRaw {
    pub pos: [f32; 4],
    pub color: [f32; 4],
    pub proj: [[f32; 4]; 4],
}

unsafe impl Pod for LightRaw {}
unsafe impl Zeroable for LightRaw {}

impl Light {
    pub fn to_raw(&self) -> LightRaw {
        use cgmath::{Deg, EuclideanSpace, Matrix4, PerspectiveFov, Point3, Vector3};
        let mx_view = Matrix4::look_at(self.pos, Point3::origin(), Vector3::unit_y());
        let projection = PerspectiveFov {
            fovy: Deg(self.fov).into(),
            aspect: 1.0,
            near: self.depth.start,
            far: self.depth.end,
        };
        let mx_view_proj = cgmath::Matrix4::from(projection.to_perspective()) * mx_view;
        LightRaw {
            pos: [self.pos.x, self.pos.y, self.pos.z, 1.0],
            color: [
                self.color.r as f32,
                self.color.g as f32,
                self.color.b as f32,
                1.0,
            ],
            proj: *mx_view_proj.as_ref(),
        }
    }
}
