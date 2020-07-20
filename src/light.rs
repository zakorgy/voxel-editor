use bytemuck::{Pod, Zeroable};
use cgmath;

pub struct Light {
    pub pos: cgmath::Point3<f32>,
    pub color: wgpu::Color,
}

#[repr(C)]
#[derive(Clone, Copy)]
pub struct LightRaw {
    pub pos: [f32; 4],
    pub color: [f32; 4],
}

unsafe impl Pod for LightRaw {}
unsafe impl Zeroable for LightRaw {}

impl Light {
    pub fn to_raw(&self) -> LightRaw {
        LightRaw {
            pos: [self.pos.x, self.pos.y, self.pos.z, 1.0],
            color: [
                self.color.r as f32,
                self.color.g as f32,
                self.color.b as f32,
                1.0,
            ],
        }
    }
}
