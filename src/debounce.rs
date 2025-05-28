use embassy_time::{Duration, Instant};

pub const DEBOUNCE_TIME: u64 = 5;

#[derive(Copy, Clone, Debug)]
pub struct Debouncer {
    state: bool,
    debounced: Option<Instant>,
}

impl Debouncer {
    pub const fn default() -> Debouncer {
        Self {
            state: false,
            debounced: None,
        }
    }
    /// Returns the pressed status of the position
    pub fn is_pressed(&self) -> bool {
        self.state
    }

    /// Updates the buf of the key. Updating the buf will also update
    /// the value returned from the is_pressed function
    pub fn update_buf(&mut self, buf: bool) {
        match self.debounced {
            Some(time) => {
                if time.elapsed() > Duration::from_millis(DEBOUNCE_TIME) {
                    self.state = buf;
                    self.debounced = None;
                }
            }
            None => {
                if buf != self.state {
                    self.debounced = Some(Instant::now());
                }
            }
        }
    }
}
