use std::time;

pub struct FpsCounter {
    instant: time::Instant,
    frame_counter: usize,
}

impl FpsCounter {
    pub fn init() -> Self {
        FpsCounter {
            instant: time::Instant::now(),
            frame_counter: 0,
        }
    }

    pub fn incr_frame(&mut self) {
        self.frame_counter += 1;
    }

    pub fn get_fps(&mut self) -> Option<f32> {
        let elapsed = self.instant.elapsed();
        if elapsed > time::Duration::from_secs(1) {
            let fps = self.frame_counter as f32 / elapsed.as_secs() as f32;
            self.reset();
            return Some(fps);
        }
        None
    }

    fn reset(&mut self) {
        *self = FpsCounter::init();
    }
}
