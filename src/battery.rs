use std::time::Instant;

/// Helper struct to keep energy data together
pub struct BatteryStatus {
    pub timestamp: Option<Instant>,
    pub energy_available: Option<f32>,
    pub energy_rate: Option<f32>,
}

impl BatteryStatus {
    pub fn new() -> BatteryStatus {
        BatteryStatus {
            timestamp: None,
            energy_available: None,
            energy_rate: None,
        }
    }

    /// Update internal values with new data
    pub fn update(&mut self, timestamp: Instant, energy_available: f32) {
        if let Some(last_ts) = self.timestamp {
            let dt = timestamp.duration_since(last_ts);
            if let Some(last_energy) = self.energy_available {
                if last_energy == energy_available {
                    // TODO: set to 0 if no change for a given time
                    return; // no change
                }
                let dt = dt.as_secs() as f32 + dt.subsec_nanos() as f32 * 1e-9;
                let e_rate = (energy_available - last_energy) / dt;
                self.energy_rate = Some(e_rate);
            }
        }
        self.timestamp = Some(timestamp);
        self.energy_available = Some(energy_available);
    }
}
