use bevy::prelude::*;
use std::time::Instant;

#[derive(Resource)] // Mark as a Bevy resource
pub struct Timer {
    start_time: Instant,
    frame_count: u32,
    fps: f64,
}

impl Timer {
    pub fn new(fps: f64) -> Self {
        println!("Creating a new Timer resource with a target FPS of {}", fps);
        Self {
            start_time: Instant::now(),
            frame_count: 0,
            fps,
        }
    }

    pub fn increment_frame(&mut self) {
        self.frame_count += 1;
    }

    pub fn stop_and_report(&self) {
        let end_time = Instant::now();
        let run_time = end_time.duration_since(self.start_time).as_secs_f64();
        let expected_time = self.frame_count as f64 / self.fps;

        println!("Ran faster by a factor of {:.2}", expected_time / run_time);
    }
}

impl Drop for Timer {
    fn drop(&mut self) {
        self.stop_and_report();
    }
}