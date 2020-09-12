use camera_controllers::{CameraPerspective, Keys, OrbitZoomCamera, OrbitZoomCameraSettings};
use cgmath::Transform;
use winit::event;

pub struct CameraWrapper {
    /// Orbiting camera implementation
    pub camera: OrbitZoomCamera<f32>,

    /// Perpective camera setting
    cam_persp: CameraPerspective<f32>,

    /// Which button to press to orbit with mouse
    orbit_button: event::MouseButton,

    /// Which button to press to zoom with mouse
    zoom_button: event::VirtualKeyCode,

    /// The x position of the mouse
    x_axis: f32,

    /// The y postion of the mouse
    y_axis: f32,
}

impl CameraWrapper {
    pub fn new(aspect_ratio: f32, meshes: f32) -> Self {
        let mut camera = OrbitZoomCamera::new(
            [0.5 * meshes, 0.5 * meshes, 0.5 * meshes],
            OrbitZoomCameraSettings::default().zoom_speed(0.4 * meshes),
        );
        camera.distance = 2.0 * meshes;
        CameraWrapper {
            camera,
            cam_persp: CameraPerspective {
                fov: 45.0f32,
                near_clip: 0.1,
                far_clip: 10.0 * meshes,
                aspect_ratio,
            },
            orbit_button: event::MouseButton::Right,
            zoom_button: event::VirtualKeyCode::LControl,
            x_axis: 0.0,
            y_axis: 0.0,
        }
    }

    /// Generates the MVP matrix for the current camera setting
    pub fn mvp_matrices(&mut self, aspect_ratio: f32) -> [[[f32; 4]; 4]; 3] {
        self.cam_persp.aspect_ratio = aspect_ratio;

        let model = cgmath::Matrix4::one();
        let view = self.camera.camera(0.0).orthogonal();
        let proj = self.cam_persp.projection();

        [model.into(), view, proj]
    }

    pub fn model_view_mat(&self) -> cgmath::Matrix4<f32> {
        self.camera.camera(0.0).orthogonal().into()
    }

    pub fn projection_mat(&self) -> cgmath::Matrix4<f32> {
        self.cam_persp.projection().into()
    }

    /// Respond to scroll and key press/release events
    pub fn update(&mut self, window_event: &winit::event::WindowEvent) -> bool {
        let mut viewport_changed = false;
        match window_event {
            event::WindowEvent::KeyboardInput {
                input:
                    event::KeyboardInput {
                        virtual_keycode,
                        state,
                        ..
                    },
                ..
            } => {
                if let Some(button) = virtual_keycode {
                    if *button == self.zoom_button {
                        match state {
                            event::ElementState::Pressed => self.camera.keys.insert(Keys::ZOOM),
                            event::ElementState::Released => self.camera.keys.remove(Keys::ZOOM),
                        }
                    }
                }
            }
            event::WindowEvent::MouseInput { state, button, .. } => {
                if *button == self.orbit_button {
                    match state {
                        event::ElementState::Pressed => self.camera.keys.insert(Keys::ORBIT),
                        event::ElementState::Released => self.camera.keys.remove(Keys::ORBIT),
                    }
                }
            }
            event::WindowEvent::MouseWheel {
                delta: event::MouseScrollDelta::LineDelta(dx, dy),
                ..
            } => {
                if self.camera.keys.contains(Keys::ZOOM) {
                    self.camera.control_camera(-*dx / 10.0, -*dy / 10.0);
                } else {
                    self.camera.control_camera(*dx * 10.0, *dy * 10.0);
                }
                viewport_changed = true;
            }
            event::WindowEvent::CursorMoved { position, .. } => {
                let dx = position.x as f32 - self.x_axis;
                self.x_axis = position.x as f32;

                let dy = position.y as f32 - self.y_axis;
                self.y_axis = position.y as f32;

                if self.camera.keys.contains(Keys::ORBIT) {
                    self.camera.control_camera(-dx / 10.0, -dy / 10.0);
                    viewport_changed = true;
                }
            }
            other => {
                log::info!("Event {:?}", other);
            }
        }
        viewport_changed
    }
}
