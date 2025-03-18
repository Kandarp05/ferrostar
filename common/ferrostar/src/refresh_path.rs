use std::time::{Duration, SystemTime};

/// Determines when to check if a better route is available
/// 
/// This allows SDK to periodically verify that the current route is still the optimal route
#[derive(Clone)]
#[cfg_attr(feature = "uniffi", derive(uniffi::Enum))]
#[cfg_attr(feature = "wasm-bindgen", derive(Deserialize, Tsify))]
#[cfg_attr(feature = "wasm-bindgen", tsify(from_wasm_abi))]
pub enum RouteRefreshStrategy {
    /// Never check for a better route
    None,

    /// Check for a better route every `min_interval_seconds`
    Periodic {
        /// The minimum interval between route refreshes in seconds
        min_interval_seconds: u64,
    },
    
    #[cfg(not(feature = "uniffi"))]
    Custom {
        detector: Box<dyn RouteRefreshDetector>,
    },
}

/// The state of the route refresh detector
/// 
/// This is used to determine if a route refresh is needed or not
#[derive(Debug, Clone, PartialEq, uniffi::Enum)]
#[cfg_attr(feature = "wasm-bindgen", test)]
#[derive(serde::Deserialize)]
#[derive(serde::Serialize)]
pub enum RouteRefreshState {
    /// No route refresh is needed
    NoRefreshNeeded,

    /// A route refresh is needed - the application should request a new route
    /// and update the navigation controller with it
    RefreshNeeded
}

/// A trait for custom route refresh detectors
/// 
/// This allows users to implement their own route refresh detectors 
pub trait RouteRefreshDetector: Send + Sync {
    fn check_route_refresh(
        &self,
        last_check_time: SystemTime,
    ) -> RouteRefreshState;
}

/// The default implementation of the route refresh detector
/// 
/// This is used to determine if a route refresh is needed or not based on the strategy
impl RouteRefreshStrategy {
    /// Check if a route refresh is needed based on the chosed strategy
    pub fn check_route_refresh(
        &self,
        last_check_time: SystemTime,
    ) -> RouteRefreshState {
        match self {
            // Never check for a better route
            RouteRefreshStrategy::None => RouteRefreshState::NoRefreshNeeded,
    
            // Check for a better route every `min_interval_seconds`
            RouteRefreshStrategy::Periodic { min_interval_seconds } => {
                let elapsed_time = Self::calculate_elapsed_time(last_check_time);
    
                if elapsed_time.as_secs() >= *min_interval_seconds {
                    RouteRefreshState::RefreshNeeded
                } else {
                    RouteRefreshState::NoRefreshNeeded
                }
            },
        }
    }
    
    /// Helper function to calculate the elapsed time since the last route refresh
    fn calculate_elapsed_time(
        last_check_time: SystemTime,
    ) -> Duration {
        SystemTime::now()
            .duration_since(last_check_time)
            .unwrap_or_else(|_| Duration::from_secs(0))
    }
}