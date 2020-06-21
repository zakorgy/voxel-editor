use cgmath;
use quaternion;
use quaternion::Quaternion;

use winit::event;

bitflags!(
    pub struct Keys: u8 {
        const ZOOM  = 0b00000001;
        const PAN   = 0b00000010;
        const ORBIT = 0b00000100;
    }
);

/// Specifies key bindings and speed modifiers for OrbitZoomCamera
pub struct OrbitZoomCameraSettings {

    /// Which button to press to orbit with mouse
    pub orbit_button: event::MouseButton,

    /// Which button to press to zoom with mouse
    pub zoom_button: event::VirtualKeyCode,

    /// Which button to press to pan with mouse
    pub pan_button: event::VirtualKeyCode,

    /// Modifier for orbiting speed (arbitrary unit)
    pub orbit_speed: f32,

    /// Modifier for pitch speed relative to orbiting speed (arbitrary unit).
    /// To reverse pitch direction, set this to -1.
    pub pitch_speed: f32,

    /// Modifier for panning speed (arbitrary unit)
    pub pan_speed: f32,

    /// Modifier for zoom speed (arbitrary unit)
    pub zoom_speed: f32,
}

impl OrbitZoomCameraSettings {

    /// Clicking and dragging OR two-finger scrolling will orbit camera,
    /// with LShift as pan modifer and LCtrl as zoom modifier
    pub fn default() -> OrbitZoomCameraSettings {
        OrbitZoomCameraSettings {
            orbit_button : event::MouseButton::Left,
            zoom_button : event::VirtualKeyCode::LControl,
            pan_button : event::VirtualKeyCode::LShift,
            orbit_speed: 0.05,
            pitch_speed: 0.1,
            pan_speed: 0.1,
            zoom_speed: 0.1,
        }
    }

    /// Set the button for orbiting
    pub fn orbit_button(self, button: event::MouseButton) -> OrbitZoomCameraSettings {
        OrbitZoomCameraSettings {
            orbit_button: button,
            .. self
        }
    }

    /// Set the button for zooming
    pub fn zoom_button(self, button: event::VirtualKeyCode) -> OrbitZoomCameraSettings {
        OrbitZoomCameraSettings {
            zoom_button: button,
            .. self
        }
    }

    /// Set the button for panning
    pub fn pan_button(self, button: event::VirtualKeyCode) -> OrbitZoomCameraSettings {
        OrbitZoomCameraSettings {
            pan_button: button,
            .. self
        }
    }

    /// Set the orbit speed modifier
    pub fn orbit_speed(self, s: f32) -> OrbitZoomCameraSettings {
        OrbitZoomCameraSettings {
            orbit_speed: s,
            .. self
        }
    }

    /// Set the pitch speed modifier
    pub fn pitch_speed(self, s: f32) -> OrbitZoomCameraSettings {
        OrbitZoomCameraSettings {
            pitch_speed: s,
            .. self
        }
    }

    /// Set the pan speed modifier
    pub fn pan_speed(self, s: f32) -> OrbitZoomCameraSettings {
        OrbitZoomCameraSettings {
            pan_speed: s,
            .. self
        }
    }

    /// Set the zoom speed modifier
    pub fn zoom_speed(self, s: f32) -> OrbitZoomCameraSettings {
        OrbitZoomCameraSettings {
            zoom_speed: s,
            .. self
        }
    }
}

/// A 3dsMax / Blender-style camera that orbits around a target point
pub struct OrbitZoomCamera {

    /// origin of camera rotation
    pub target: cgmath::Vector3<f32>,

    /// Rotation of camera
    pub rotation: Quaternion<f32>,

    /// Pitch up/down from target
    pub pitch: f32,

    /// Yaw left/right from target
    pub yaw: f32,

    /// camera distance from target
    pub distance: f32,

    /// Settings for the camera
    pub settings: OrbitZoomCameraSettings,

    /// Current keys that are pressed
    keys: Keys,

    /// State of the cursor
    xAxis: f32,
    yAxis: f32,
}


impl OrbitZoomCamera {
    /// Create a new OrbitZoomCamera targeting the given coordinates
    pub fn new(target: [f32; 3], settings: OrbitZoomCameraSettings) -> OrbitZoomCamera {
        OrbitZoomCamera {
            target: target.into(),
            rotation: quaternion::id(),
            distance: 2.0,
            pitch: -2.0,
            yaw: 1.5,
            keys: Keys::empty(),
            xAxis: 0.0,
            yAxis: 0.0,
            settings: settings
        }
    }

    /// Orbit the camera using the given horizontal and vertical params,
    /// or zoom or pan if the appropriate modifier keys are pressed
    fn control_camera(&mut self, dx: f32, dy: f32) {

        let _1 = 1.0;
        let _0 = 0.0;

        if self.keys.contains(Keys::PAN) {
            log::debug!("Pan dx {:?}, dy {:?}", dx, dy);
            // Pan target position along plane normal to camera direction
            let dx = dx * self.settings.pan_speed;
            let dy = dy * self.settings.pan_speed;

            let right: cgmath::Vector3<f32> = quaternion::rotate_vector(self.rotation, [_1, _0, _0]).into();
            let up: cgmath::Vector3<f32> = quaternion::rotate_vector(self.rotation, [_0, _1, _0]).into();
            self.target =
                self.target + (up * dy) +
                (right * dx);

        } else if self.keys.contains(Keys::ZOOM) {
            log::debug!("Zoom dx {:?}, dy {:?}", dx, dy);
            // Zoom to / from target
            self.distance = self.distance + dy * self.settings.zoom_speed;

        } else if self.keys.contains(Keys::ORBIT) {
            log::debug!("Orbit dx {:?}, dy {:?}", dx, dy);
            // Orbit around target
            let dx = dx * self.settings.orbit_speed;
            let dy = dy * self.settings.orbit_speed;

            self.yaw = self.yaw + dx;
            self.pitch = self.pitch + dy*self.settings.pitch_speed;
            self.rotation = quaternion::mul(
                quaternion::axis_angle([_0, _1, _0], self.yaw),
                quaternion::axis_angle([_1, _0, _0], self.pitch)
            );

        }
    }

    /// Respond to scroll and key press/release events
    pub fn update(&mut self, window_event: &winit::event::WindowEvent) {
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
                    if *button == self.settings.zoom_button {
                        match state {
                            event::ElementState::Pressed => self.keys.insert(Keys::ZOOM),
                            event::ElementState::Released => self.keys.remove(Keys::ZOOM),
                        }
                    }
                    if *button == self.settings.pan_button {
                        match state {
                            event::ElementState::Pressed => self.keys.insert(Keys::PAN),
                            event::ElementState::Released => self.keys.remove(Keys::PAN),
                        }
                    }
                }
            },
            event::WindowEvent::MouseInput {
                state,
                button,
                ..
            } => {
                if *button == self.settings.orbit_button {
                    match state {
                        event::ElementState::Pressed => self.keys.insert(Keys::ORBIT),
                        event::ElementState::Released => self.keys.remove(Keys::ORBIT),
                    }
                }
            },
            event::WindowEvent::MouseWheel {
                delta: event::MouseScrollDelta::LineDelta(dx, dy),
                ..
            } => self.control_camera(*dx, *dy),
            event::WindowEvent::AxisMotion {
                axis,
                value,
                ..
            } => {
                let mut dx = 0.0;
                let mut dy = 0.0;
                match axis {
                    0 => {
                        dx = (*value as f32) - self.xAxis;
                        self.xAxis = (*value as f32);
                    }
                    1 => {
                        dy = (*value as f32) - self.yAxis;
                        self.yAxis = (*value as f32);
                    }
                    _ => {}
                }
                self.control_camera(dx, dy)
            }
            other => {
                //println!("Event {:?}", other);
            }
        }
    }

    pub fn generate_matrix(&self, aspect_ratio: f32) -> cgmath::Matrix4<f32> {
        let mx_projection = cgmath::perspective(cgmath::Deg(45f32), aspect_ratio, 0.1, 10.0);
        let mx_view = cgmath::Matrix4::look_at(
            //cgmath::Point3::new(1.5f32, -2.0, 2.0),
            cgmath::Point3::new(self.yaw, self.pitch, self.distance),
            //cgmath::Point3::new(0f32, 1.0, 0.0),
            cgmath::Point3::new(self.target.x, self.target.y, self.target.z),
            cgmath::Vector3::unit_z(),
        );
        return mx_projection * mx_view
    }
}

pub struct Camera {
    fov: f32,
    znear: f32,
    zfar: f32,
    rotation: cgmath::Vector3<f32>,
    rotation_speed: f32,
    zoom_speed: f32,
    position: cgmath::Vector3<f32>,
    view_pos: cgmath::Vector4<f32>,
    perspective: cgmath::Matrix4<f32>,
    view: cgmath::Matrix4<f32>,
    orbit_button : event::MouseButton,
    zoom_button : event::VirtualKeyCode,
    keys: Keys,
}

impl Camera {
    fn update_view_matrix(&mut self){

    }

    pub fn near_clip(&self) -> f32 {
        self.znear
    }

    pub fn far_clip(&self) -> f32 {
        self.zfar
    }

    pub fn set_perspective(&mut self, fov: f32, aspect: f32, znear: f32, zfar: f32) {
        self.fov = fov;
        self.znear = znear;
        self.zfar = zfar;
        self.perspective = cgmath::perspective(cgmath::Deg(fov), aspect, znear, zfar);
    }

    pub fn update_aspect_ratio(&mut self, aspect: f32)
    {
        self.perspective = cgmath::perspective(cgmath::Deg(self.fov), aspect, self.znear, self.zfar);
    }

    pub fn set_position(&mut self, position: cgmath::Vector3<f32>)
    {
        self.position = position;
        self.update_view_matrix();
    }

    pub fn set_rotation(&mut self, rotation: cgmath::Vector3<f32>)
    {
        self.rotation = rotation;
        self.update_view_matrix();
    }

    pub fn rotate(&mut self, delta: cgmath::Vector3<f32>)
    {
        self.rotation += delta;
        self.update_view_matrix();
    }

    void set_translation(glm::vec3 translation)
    {
        self.position = translation;
        self.update_view_matrix();
    };

    void translate(glm::vec3 delta)
    {
        self.position += delta;
        self.update_view_matrix();
    }

    void setRotationSpeed(rotationSpeed: f32)
    {
        self.rotationSpeed = rotationSpeed;
    }

    void setMovementSpeed(movementSpeed: f32)
    {
        self.movementSpeed = movementSpeed;
    }
}
