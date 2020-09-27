use bytemuck::{Pod, Zeroable};
use cgmath::{EuclideanSpace, Matrix4, Ortho, Point3, Vector3};
use iced_wgpu::wgpu;

pub struct Light {
    pub pos: Point3<f32>,
    pub color: wgpu::Color,
    pub fov: f32,
    pub depth: std::ops::Range<f32>,
}

#[repr(C)]
#[derive(Clone, Copy)]
pub struct LightRaw {
    pub direction: [f32; 4],
    pub color: [f32; 4],
    pub proj: [[f32; 4]; 4],
}

unsafe impl Pod for LightRaw {}
unsafe impl Zeroable for LightRaw {}

impl Light {
    pub fn to_raw(&self, mesh_count: f32) -> LightRaw {
        let origin = Point3::origin();
        let mx_view = Matrix4::look_at(self.pos, origin, Vector3::unit_y());
        let ortho_projection = Ortho {
            left: -mesh_count,
            right: mesh_count,
            bottom: -mesh_count,
            top: mesh_count,
            near: -mesh_count,
            far:  3.0 * mesh_count,
        };
        let mx_view_proj = cgmath::Matrix4::from(ortho_projection) * mx_view;
        let light_dir = self.pos - origin;
        LightRaw {
            direction: [light_dir.x, light_dir.y, light_dir.z, 1.0],
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
