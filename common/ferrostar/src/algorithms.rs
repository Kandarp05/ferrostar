//! Common spatial algorithms which are useful for navigation.

use crate::{
    models::CourseOverGround,
    navigation_controller::models::{
        StepAdvanceMode,
        StepAdvanceStatus::{self, Advanced, EndOfRoute},
    },
};
use crate::{
    models::{GeographicCoordinate, RouteStep, UserLocation},
    navigation_controller::models::{SpecialAdvanceConditions, TripProgress},
};
use geo::{
    Bearing, Closest, Coord, Distance, Euclidean, Geodesic, Haversine, HaversineClosestPoint, Length, LineLocatePoint, LineString, Point
};

#[cfg(test)]
use {
    crate::navigation_controller::test_helpers::gen_dummy_route_step,
    geo::{coord, point, CoordsIter},
    proptest::{collection::vec, prelude::*},
};

#[cfg(all(test, feature = "std", not(feature = "web-time")))]
use std::time::SystemTime;
#[cfg(all(test, feature = "web-time"))]
use web_time::SystemTime;

/// Enum for calcuation methods
/// 
/// The `Haversine` method is a simple formula that calculates the distance between two points on a sphere.
/// The `Geodesic` method uses the Vincenty formula to calculate the distance between two points on an ellipsoid.
/// The `Adaptive` method uses evidence-based approach to dynamically switch between the two methods.
#[derive(Clone, Debug)]
pub enum DistanceCalculation {
    ///Always use the Haversine formula
    Haversine,
    ///Always use the Geodesic formula
    Geodesic,
    ///Adaptively choose between algorithms based on conditions
    Adaptive {
        ///Current algorithm in use
        current_algorithm: AdaptiveAlgorithm,
        ///Accumulated evidence for switching (positive = switch to Geodesic, negative = switch to Haversine)
        evidence: f64,
        /// Threshold for switching 
        switch_threshold: f64,
        /// Evidence decay rate
        evidence_decay: f64,
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum AdaptiveAlgorithm {
    Haversine,
    Geodesic,
}

#[derive(Debug, Clone, Copy)]
struct LatitudeThresholds {
    high: f64,
    moderate: f64,
    mid: f64,
    equatorial: f64,
}

#[derive(Debug, Clone, Copy)]
struct DistanceThresholds {
    long: f64,    
    medium: f64,  
    short: f64,   
}

#[derive(Debug, Clone, Copy)]
struct DecayMultipliers {
    equatorial: f64,
    high_latitude: f64, 
    mid_latitude: f64,
    negative_evidence_boost: f64,
}

impl DistanceCalculation {
    ///Change these values according to need.

    const LATITUDE: LatitudeThresholds = LatitudeThresholds {
        high: 65.0,
        moderate: 55.0,
        mid: 40.0,
        equatorial: 15.0,
    };

    const DISTANCE: DistanceThresholds = DistanceThresholds {
        long: 500_000.0, // 500km
        medium: 200_000.0, // 200km
        short: 5_000.0, // 5km
    };

    const DECAY: DecayMultipliers = DecayMultipliers {
        equatorial: 7.0,
        high_latitude: 4.5,
        mid_latitude: 3.0,
        negative_evidence_boost: 1.5,
    };

    /// Calculate distance between two points using the current algorithm choice.
    /// 
    /// For the Adaptive variant, this method accumulates evidence based on the current conditions and will switch algorithms if evidence passes a threshold.
    /// 
    /// This method returns both the calculated distance and the updated DistanceCalculation enum.
    pub fn calculate_distance(&self, from: Point, to: Point) -> (f64, DistanceCalculation) {
        if !is_valid_float(from.x()) || !is_valid_float(from.y()) || !is_valid_float(to.x()) || !is_valid_float(to.y()) {
            return (0.0, self.clone());
        }
        
        match self {
            Self::Haversine => (Haversine::distance(from, to), self.clone()),
            Self::Geodesic => (Geodesic::distance(from, to), self.clone()),
            Self::Adaptive { current_algorithm, evidence, switch_threshold, evidence_decay } => {
                let mut new_evidence = *evidence;

                // Factor 1: Latitude -> Higher latitude favor Geodesic
                let (latitude_factor, max_latitude) = self.calculate_latitude_evidence(from, to);

                // Factor 2: Distance -> Longer distances favor Geodesic
                let distance_factor = self.calculate_distance_evidence(from, to);
                

                // Apply aggressive decay when using Geodesic to quickly switch back when possible
                if *current_algorithm == AdaptiveAlgorithm::Geodesic {
                    new_evidence = self.apply_evidence_decay(*evidence, max_latitude, *evidence_decay)
                } else {
                    // Standard decay in Haversine mode
                    new_evidence *= 1.0 - evidence_decay;
                }
                
                // Add new factors
                new_evidence += latitude_factor + distance_factor;

                // Speed up switching back to Haversine when evidence suggests it's appropriate
                if new_evidence < 0.0 && *current_algorithm == AdaptiveAlgorithm::Geodesic {
                    new_evidence *= Self::DECAY.negative_evidence_boost; // Boost negative evidence in Geodesic mode
                }

                let (new_algorithm, final_evidence) = self.determine_algorithm(*current_algorithm, new_evidence, *switch_threshold);

                let distance = match new_algorithm {
                    AdaptiveAlgorithm::Haversine => Haversine::distance(from, to),
                    AdaptiveAlgorithm::Geodesic => Geodesic::distance(from, to),
                };

                (
                    distance,
                    Self::Adaptive {
                        current_algorithm: new_algorithm,
                        evidence: final_evidence,
                        switch_threshold: *switch_threshold,
                        evidence_decay: *evidence_decay,
                    }
                )
            }
        }
    }

    fn calculate_latitude_evidence(&self, from: Point, to: Point) -> (f64, f64) {
        let max_latitude = from.y().abs().max(to.y().abs());
    
        let latitude_factor = if max_latitude > Self::LATITUDE.high {
            1.0 // Strong evidence for Geodesic at very high latitudes
        } else if max_latitude > Self::LATITUDE.moderate {
            0.5 // Moderate evidence at high latitudes
        } else if max_latitude > Self::LATITUDE.mid {
            0.1 // Very light evidence at mid-high latitudes
        } else if max_latitude < Self::LATITUDE.equatorial {
            -0.8 // Very strong preference for Haversine at equatorial regions
        } else {
            -0.4 // Strong preference for Haversine at mid latitudes
        };
    
        (latitude_factor, max_latitude)
    }

    fn calculate_distance_evidence(&self, from: Point, to: Point) -> f64 {
        let rough_distance = self.estimate_rough_distance(from, to);

        if rough_distance > Self::DISTANCE.long {
            0.5 // Strong evidence
        } else if rough_distance > Self::DISTANCE.medium {
            0.3 // Moderate evidence
        } else if rough_distance < Self::DISTANCE.short {
            -0.6 // Strong preference for Haversine
        } else {
            -0.3 // Preference for Haversine at medium distances
        }
    }

    fn estimate_rough_distance(&self, from: Point, to: Point) -> f64 {
        let dlat = (to.y() - from.y()).abs();
        let dlon = (to.x() - from.x()).abs();
        (dlat.powi(2) + dlon.powi(2)).sqrt() * 111_000.0 // ~111km per degree
    }

    fn apply_evidence_decay(&self, evidence: f64, max_latitude: f64, evidence_decay: f64) -> f64 {
        // Apply stronger decay at lower latitudes to switch back quickly
        let decay_multiplier = if max_latitude < 30.0 {
            Self::DECAY.equatorial
        } else if max_latitude < 45.0 {
            Self::DECAY.high_latitude
        } else {
            Self::DECAY.mid_latitude
        };

        evidence * (1.0 - (evidence_decay * decay_multiplier))
    }

    fn determine_algorithm(&self, current_algorithm: AdaptiveAlgorithm, evidence: f64, switch_threshold: f64) -> (AdaptiveAlgorithm, f64) {
        match current_algorithm {
            AdaptiveAlgorithm::Haversine => {
                if evidence > switch_threshold {
                    // Reset evidence when switching to Geodesic
                    (AdaptiveAlgorithm::Geodesic, 0.0)
                } else {
                    (AdaptiveAlgorithm::Haversine, evidence)
                }
            },
            AdaptiveAlgorithm::Geodesic => {
                // Make it easier to switch back to Haversine (only 25% of threshold)
                if evidence < -switch_threshold * 0.25 {
                    // Reset evidence when switching to Haversine
                    (AdaptiveAlgorithm::Haversine, 0.0)
                } else {
                    (AdaptiveAlgorithm::Geodesic, evidence)
                }
            }
        }
    }

    /// Create a default adaptive distance calculator with the Haversine algorithm as the initial choice.
    /// 
    /// Default setting prefer Haversine  for better performance but will switch to Geodesic for longer distances and higher latitudes.
    pub fn default_adaptive() -> Self {
        Self::Adaptive {
            current_algorithm: AdaptiveAlgorithm::Haversine, // Start with faster algorithm
            evidence: 0.0,
            switch_threshold: 1.2, // Higher threshold to avoid unnecessary switches to Geodesic
            evidence_decay: 0.1,
        }
    }
}

/// Get the index of the closest *segment* to the user's location within a [`LineString`].
///
/// A [`LineString`] is a set of points (ex: representing the geometry of a maneuver),
/// and this function identifies which segment a point is closest to,
/// so that you can correctly match attributes along a maneuver.
///
/// In the case of a location being exactly on the boundary
/// (unlikely in the real world, but quite possible in simulations),
/// the *first* segment of equal distance to the location will be matched.
///
/// The maximum value returned is *one less than* the last coordinate index into `line`.
/// Returns [`None`] if `line` contains fewer than two coordinates.
pub fn index_of_closest_segment_origin(location: UserLocation, line: &LineString) -> Option<u64> {
    let point = Point::from(location.coordinates);

    line.lines()
        // Iterate through all segments of the line
        .enumerate()
        // Find the line segment closest to the user's location
        .min_by(|(_, line_segment_1), (_, line_segment_2)| {
            // Note: lines don't implement haversine distances
            // In case you're tempted to say that this looks like cross track distance,
            // note that the Line type here is actually a line *segment*,
            // and we actually want to find the closest segment, not the closest mathematical line.
            let dist1 = Euclidean::distance(line_segment_1, &point);
            let dist2 = Euclidean::distance(line_segment_2, &point);
            dist1.total_cmp(&dist2)
        })
        .map(|(index, _)| index as u64)
}

/// Get the bearing to the next point on the `LineString`.
///
/// Returns [`None`] if the index points at or past the last point in the `LineString`.
fn get_bearing_to_next_point(
    index_along_line: usize,
    line: &LineString,
) -> Option<CourseOverGround> {
    let mut points = line.points().skip(index_along_line);

    let current = points.next()?;
    let next = points.next()?;

    let degrees = Geodesic::bearing(current, next);
    Some(CourseOverGround::new(degrees, None))
}

/// Apply a snapped course to a user location.
///
/// This function snaps the course to travel along the provided line,
/// starting from the given coordinate index along the line.
///
/// If the given index is None or out of bounds, the original location will be returned unmodified.
/// `index_along_line` is optional to improve ergonomics elsewhere in the codebase,
/// despite the API looking a little funny.
pub fn apply_snapped_course(
    location: UserLocation,
    index_along_line: Option<u64>,
    line: &LineString,
) -> UserLocation {
    let snapped_course =
        index_along_line.and_then(|index| get_bearing_to_next_point(index as usize, line));

    let course_over_ground = snapped_course.or(location.course_over_ground);

    UserLocation {
        course_over_ground,
        ..location
    }
}

/// Snaps a user location to the closest point on a route line.
///
/// If the location cannot be snapped (should only be possible with an invalid coordinate or geometry),
/// the location is returned unaltered.
pub fn snap_user_location_to_line(location: UserLocation, line: &LineString) -> UserLocation {
    let original_point = Point::from(location);

    snap_point_to_line(&original_point, line).map_or_else(
        || location,
        |snapped| UserLocation {
            coordinates: GeographicCoordinate {
                lng: snapped.x(),
                lat: snapped.y(),
            },
            ..location
        },
    )
}

/// Internal function that truncates a float to 6 digits.
///
/// Note that this approach is not a substitute for fixed precision decimals,
/// but it is acceptable for our use case,
/// where our main goal is to avoid precision issues for values which do not matter
/// and remove most edge cases with floating point numbers.
///
/// The `decimal_digits` parameter refers to the number of digits after the point.
pub(crate) fn trunc_float(value: f64, decimal_digits: u32) -> f64 {
    let factor = 10_i64.pow(decimal_digits) as f64;
    (value * factor).round() / factor
}

/// Predicate which is used to filter out several types of undesirable floating point values.
///
/// These include NaN values, subnormals (usually the result of underflow), and infinite values.
fn is_valid_float(value: f64) -> bool {
    !value.is_nan() && !value.is_subnormal() && !value.is_infinite()
}

fn snap_point_to_line(point: &Point, line: &LineString) -> Option<Point> {
    // Bail early when we have two essentially identical points.
    // This can cause some issues with edge cases (captured in proptest regressions)
    // with the underlying libraries.
    if Euclidean::distance(line, point) < 0.000_001 {
        return Some(*point);
    }

    // If either point is not a "valid" float, bail.
    if !is_valid_float(point.x()) || !is_valid_float(point.y()) {
        return None;
    }

    match line.haversine_closest_point(point) {
        Closest::Intersection(snapped) | Closest::SinglePoint(snapped) => {
            let (x, y) = (snapped.x(), snapped.y());
            if is_valid_float(x) && is_valid_float(y) {
                Some(snapped)
            } else {
                None
            }
        }
        Closest::Indeterminate => None,
    }
}

/// Calculates the distance a point is from a line (route segment), in meters.
///
/// This function should return a value for valid inputs,
/// but due to the vagaries of floating point numbers
/// (infinity and `NaN` being possible inputs),
/// we return an optional to insulate callers from edge cases.
///
/// # Example
///
/// ```
/// // Diagonal line from the origin to (1,1)
/// use geo::{coord, LineString, point};
/// use ferrostar::algorithms::deviation_from_line;
///
/// let linestring = LineString::new(vec![coord! {x: 0.0, y: 0.0}, coord! {x: 1.0, y: 1.0}]);
///
/// let origin = point! {
///     x: 0.0,
///     y: 0.0,
/// };
/// let midpoint = point! {
///     x: 0.5,
///     y: 0.5,
/// };
/// let off_line = point! {
///     x: 1.0,
///     y: 0.5,
/// };
///
/// // The origin is directly on the line
/// assert_eq!(deviation_from_line(&origin, &linestring), Some(0.0));
///
/// // The midpoint is also directly on the line
/// assert_eq!(deviation_from_line(&midpoint, &linestring), Some(0.0));
///
/// // This point, however is off the line.
/// // That's a huge number, because we're dealing with points jumping by degrees ;)
/// println!("{:?}", deviation_from_line(&off_line, &linestring));
/// assert!(deviation_from_line(&off_line, &linestring)
///     .map_or(false, |deviation| deviation - 39316.14208341989 < f64::EPSILON));
/// ```
pub fn deviation_from_line(point: &Point, line: &LineString) -> Option<f64> {
    snap_point_to_line(point, line).and_then(|snapped| {
        let distance = Haversine::distance(snapped, *point);

        if distance.is_nan() || distance.is_infinite() {
            None
        } else {
            Some(distance)
        }
    })
}

fn is_within_threshold_to_end_of_linestring(
    current_position: &Point,
    current_step_linestring: &LineString,
    threshold: f64,
) -> bool {
    current_step_linestring
        .coords()
        .last()
        .map_or(false, |end_coord| {
            let end_point = Point::from(*end_coord);
            let distance_to_end = Haversine::distance(end_point, *current_position);

            distance_to_end <= threshold
        })
}

/// Determines whether the navigation controller should complete the current route step
/// and move to the next.
///
/// NOTE: The [`UserLocation`] should *not* be snapped.
pub fn should_advance_to_next_step(
    current_step_linestring: &LineString,
    next_route_step: Option<&RouteStep>,
    user_location: &UserLocation,
    step_advance_mode: StepAdvanceMode,
) -> bool {
    let current_position = Point::from(user_location.coordinates);

    match step_advance_mode {
        StepAdvanceMode::Manual => false,
        StepAdvanceMode::DistanceToEndOfStep {
            distance,
            minimum_horizontal_accuracy,
        } => {
            if user_location.horizontal_accuracy > minimum_horizontal_accuracy.into() {
                false
            } else {
                is_within_threshold_to_end_of_linestring(
                    &current_position,
                    current_step_linestring,
                    f64::from(distance),
                )
            }
        }
        StepAdvanceMode::RelativeLineStringDistance {
            minimum_horizontal_accuracy,
            special_advance_conditions,
        } => {
            if user_location.horizontal_accuracy > minimum_horizontal_accuracy.into() {
                false
            } else {
                if let Some(condition) = special_advance_conditions {
                    match condition {
                        SpecialAdvanceConditions::AdvanceAtDistanceFromEnd(distance) => {
                            // Short-circuit: if we are close to the end of the step,
                            // we may advance early.
                            if is_within_threshold_to_end_of_linestring(
                                &current_position,
                                current_step_linestring,
                                f64::from(distance),
                            ) {
                                return true;
                            }
                        }
                        SpecialAdvanceConditions::MinimumDistanceFromCurrentStepLine(distance) => {
                            // Short-circuit: do NOT advance if we are within `distance`
                            // of the current route step.
                            //
                            // Historical note: we previously considered checking distance from the
                            // end of the current step instead, but this actually failed
                            // the self-intersecting route tests, since the step break isn't
                            // necessarily near the intersection.
                            //
                            // The last step is special and this logic does not apply.
                            if let Some(next_step) = next_route_step {
                                // Note this special next_step distance check; otherwise we get stuck at the end!
                                if next_step.distance > f64::from(distance)
                                    && deviation_from_line(
                                        &current_position,
                                        &current_step_linestring,
                                    )
                                    .map_or(true, |deviation| deviation <= f64::from(distance))
                                {
                                    return false;
                                }
                            }
                        }
                    }
                }

                if let Some(next_step) = next_route_step {
                    // FIXME: This isn't very efficient to keep doing at the moment
                    let next_step_linestring = next_step.get_linestring();

                    // Try to snap the user's current location to the current step
                    // and next step geometries
                    if let (Some(current_step_closest_point), Some(next_step_closest_point)) = (
                        snap_point_to_line(&current_position, current_step_linestring),
                        snap_point_to_line(&current_position, &next_step_linestring),
                    ) {
                        // If the user's distance to the snapped location on the *next* step is <=
                        // the user's distance to the snapped location on the *current* step,
                        // advance to the next step
                        Haversine::distance(current_position, next_step_closest_point)
                            <= Haversine::distance(current_position, current_step_closest_point)
                    } else {
                        // The user's location couldn't be mapped to a single point on both the current and next step.
                        // Fall back to the distance to end of step mode, which has some graceful fallbacks.
                        // In real-world use, this should only happen for values which are EXTREMELY close together.
                        should_advance_to_next_step(
                            current_step_linestring,
                            None,
                            user_location,
                            StepAdvanceMode::DistanceToEndOfStep {
                                distance: minimum_horizontal_accuracy,
                                minimum_horizontal_accuracy,
                            },
                        )
                    }
                } else {
                    // Trigger arrival when the user gets within a circle of the minimum horizontal accuracy
                    should_advance_to_next_step(
                        current_step_linestring,
                        None,
                        user_location,
                        StepAdvanceMode::DistanceToEndOfStep {
                            distance: minimum_horizontal_accuracy,
                            minimum_horizontal_accuracy,
                        },
                    )
                }
            }
        }
    }
}

/// Runs a state machine transformation to advance one step.
///
/// Note that this function is pure and the caller must persist any mutations
/// including dropping a completed step.
/// This function is safe and idempotent in the case that it is accidentally
/// invoked with no remaining steps.
pub(crate) fn advance_step(remaining_steps: &[RouteStep]) -> StepAdvanceStatus {
    // NOTE: The first item is the *current* step, and we want the *next* step.
    match remaining_steps.get(1) {
        Some(new_step) => Advanced {
            step: new_step.clone(),
            linestring: new_step.get_linestring(),
        },
        None => EndOfRoute,
    }
}

/// Computes the distance that a point lies along a linestring,
/// assuming that units are latitude and longitude for the geometries.
///
/// The result is given in meters.
/// The result may be [`None`] in case of invalid input such as infinite floats.
fn distance_along(point: &Point, linestring: &LineString) -> Option<f64> {
    let total_length = linestring.length::<Haversine>();
    if total_length == 0.0 {
        return Some(0.0);
    }

    let (_, _, traversed) = linestring.lines().try_fold(
        (0f64, f64::INFINITY, 0f64),
        |(cum_length, closest_dist_to_point, traversed), segment| {
            // Convert to a LineString so we get haversine ops
            let segment_linestring = LineString::from(segment);

            // Compute distance to the line (sadly Euclidean only; no haversine_distance in GeoRust
            // but this is probably OK for now)
            let segment_distance_to_point = Euclidean::distance(&segment, point);
            // Compute total segment length in meters
            let segment_length = segment_linestring.length::<Haversine>();

            if segment_distance_to_point < closest_dist_to_point {
                let segment_fraction = segment.line_locate_point(point)?;
                Some((
                    cum_length + segment_length,
                    segment_distance_to_point,
                    cum_length + segment_fraction * segment_length,
                ))
            } else {
                Some((
                    cum_length + segment_length,
                    closest_dist_to_point,
                    traversed,
                ))
            }
        },
    )?;
    Some(traversed)
}

/// Computes the distance between a location and the end of the current route step.
/// We assume that input location is pre-snapped to route step's linestring,
/// and that travel is proceeding along the route (not straight line distance).
///
/// The result may be [`None`] in case of invalid input such as infinite floats.
fn travel_distance_to_end_of_step(
    snapped_location: &Point,
    current_step_linestring: &LineString,
) -> Option<f64> {
    let step_length = current_step_linestring.length::<Haversine>();
    distance_along(snapped_location, current_step_linestring)
        .map(|traversed| step_length - traversed)
}

/// Computes the user's progress along the current trip (distance to destination, ETA, etc.).
///
/// NOTE to callers: `remaining_steps` includes the current step!
pub fn calculate_trip_progress(
    snapped_location: &Point,
    current_step_linestring: &LineString,
    remaining_steps: &[RouteStep],
) -> TripProgress {
    let Some(current_step) = remaining_steps.first() else {
        return TripProgress {
            distance_to_next_maneuver: 0.0,
            distance_remaining: 0.0,
            duration_remaining: 0.0,
        };
    };

    // Calculate the distance and duration till the end of the current route step.
    let distance_to_next_maneuver =
        travel_distance_to_end_of_step(snapped_location, current_step_linestring)
            .unwrap_or(current_step.distance);

    // This could be improved with live traffic data along the route.
    // TODO: Figure out the best way to enable this use case
    let pct_remaining_current_step = if current_step.distance > 0f64 {
        distance_to_next_maneuver / current_step.distance
    } else {
        0f64
    };

    // Get the percentage of duration remaining in the current step.
    let duration_to_next_maneuver = pct_remaining_current_step * current_step.duration;

    // Exit early if there is only the current step:
    if remaining_steps.len() == 1 {
        return TripProgress {
            distance_to_next_maneuver,
            distance_remaining: distance_to_next_maneuver,
            duration_remaining: duration_to_next_maneuver,
        };
    }

    let steps_after_current = &remaining_steps[1..];
    let distance_remaining = distance_to_next_maneuver
        + steps_after_current
            .iter()
            .map(|step| step.distance)
            .sum::<f64>();

    let duration_remaining = duration_to_next_maneuver
        + steps_after_current
            .iter()
            .map(|step| step.duration)
            .sum::<f64>();

    TripProgress {
        distance_to_next_maneuver,
        distance_remaining,
        duration_remaining,
    }
}

/// Convert a vector of geographic coordinates to a [`LineString`].
pub(crate) fn get_linestring(geometry: &[GeographicCoordinate]) -> LineString {
    geometry
        .iter()
        .map(|coord| Coord {
            x: coord.lng,
            y: coord.lat,
        })
        .collect()
}

#[cfg(test)]
/// Creates a user location at the given coordinates,
/// with all other values set to defaults or (in the case of the timestamp), the current time.
fn make_user_location(lng: f64, lat: f64) -> UserLocation {
    UserLocation {
        coordinates: GeographicCoordinate { lng, lat },
        horizontal_accuracy: 0.0,
        course_over_ground: None,
        timestamp: SystemTime::now(),
        speed: None,
    }
}

#[cfg(test)]
prop_compose! {
    fn arb_coord()(x in -180f64..180f64, y in -90f64..90f64) -> Coord {
        coord! {x: x, y: y}
    }
}

#[cfg(test)]
proptest! {
    #[test]
    fn snap_point_to_line_intersection(
        x1: f64, y1: f64,
        x2: f64, y2: f64,
    ) {
        let point = point! {
            x: x1,
            y: y1,
        };
        let line = LineString::new(vec! {
            coord! {
                x: x1,
                y: y1,
            },
            coord! {
                x: x2,
                y: y2,
            },
        });

        if let Some(snapped) = snap_point_to_line(&point, &line) {
            let x = snapped.x();
            let y = snapped.y();

            prop_assert!(is_valid_float(x) || (!is_valid_float(x1) && x == x1));
            prop_assert!(is_valid_float(y) || (!is_valid_float(y1) && y == y1));

            prop_assert!(Euclidean::distance(&line, &snapped) < 0.000001);
        } else {
            // Edge case 1: extremely small differences in values
            let is_miniscule_difference = (x1 - x2).abs() < 0.00000001 || (y1 - y2).abs() < 0.00000001;
            // Edge case 2: Values which are clearly not WGS84 ;)
            let is_non_wgs84 = (x1 - x2).abs() > 180.0 || (y1 - y2).abs() > 90.0;
            prop_assert!(is_miniscule_difference || is_non_wgs84);
        }
    }

    #[test]
    fn should_advance_exact_position(
        x1: f64, y1: f64,
        x2: f64, y2: f64,
        x3: f64, y3: f64,
        has_next_step: bool,
        distance: u16, minimum_horizontal_accuracy: u16, excess_inaccuracy in 0f64..,
        threshold: Option<u16>,
    ) {
        if !(x1 == x2 && y1 == y2) && !(x1 == x3 && y1 == y3) {
            // Guard against:
            //   1. Invalid linestrings
            //   2. Invalid tests (we assume that the route isn't a closed loop)
            let current_route_step = gen_dummy_route_step(x1, y1, x2, y2);
            let next_route_step = if has_next_step {
                Some(gen_dummy_route_step(x2, y2, x3, y3))
            } else {
                None
            };
            let exact_user_location = UserLocation {
                coordinates: *current_route_step.geometry.last().unwrap(), // Exactly at the end location
                horizontal_accuracy: 0.0,
                course_over_ground: None,
                timestamp: SystemTime::now(),
                speed: None
            };

            let inaccurate_user_location = UserLocation {
                horizontal_accuracy: (minimum_horizontal_accuracy as f64) + excess_inaccuracy,
                ..exact_user_location
            };

            // Never advance to the next step when StepAdvanceMode is Manual
            prop_assert!(!should_advance_to_next_step(&current_route_step.get_linestring(), next_route_step.as_ref(), &exact_user_location, StepAdvanceMode::Manual));
            prop_assert!(!should_advance_to_next_step(&current_route_step.get_linestring(), next_route_step.as_ref(), &inaccurate_user_location, StepAdvanceMode::Manual));

            // Always succeeds in the base case in distance to end of step mode
            let cond = should_advance_to_next_step(&current_route_step.get_linestring(), next_route_step.as_ref(), &exact_user_location, StepAdvanceMode::DistanceToEndOfStep {
                distance, minimum_horizontal_accuracy
            });
            prop_assert!(cond);

            // Same when looking at the relative distances between the two step geometries
            let cond = should_advance_to_next_step(&current_route_step.get_linestring(), next_route_step.as_ref(), &exact_user_location, StepAdvanceMode::RelativeLineStringDistance {
                minimum_horizontal_accuracy,
                special_advance_conditions: threshold.map(|distance| SpecialAdvanceConditions::AdvanceAtDistanceFromEnd(distance))
            });
            prop_assert!(cond);

            // Should always fail (unless excess_inaccuracy is zero), as the horizontal accuracy is worse than (>) than the desired error threshold
            prop_assert_eq!(should_advance_to_next_step(&current_route_step.get_linestring(), next_route_step.as_ref(), &inaccurate_user_location, StepAdvanceMode::DistanceToEndOfStep {
                distance, minimum_horizontal_accuracy
            }), excess_inaccuracy == 0.0, "Expected that the navigation would not advance to the next step except when excess_inaccuracy is 0");
            prop_assert_eq!(should_advance_to_next_step(&current_route_step.get_linestring(), next_route_step.as_ref(), &inaccurate_user_location, StepAdvanceMode::RelativeLineStringDistance {
                minimum_horizontal_accuracy,
                special_advance_conditions: threshold.map(|distance| SpecialAdvanceConditions::AdvanceAtDistanceFromEnd(distance))
            }), excess_inaccuracy == 0.0, "Expected that the navigation would not advance to the next step except when excess_inaccuracy is 0");
        }
    }

    #[test]
    fn should_advance_inexact_position(
        x1: f64, y1: f64,
        x2: f64, y2: f64,
        x3: f64, y3: f64,
        error in -0.003f64..=0.003f64, has_next_step: bool,
        distance: u16, minimum_horizontal_accuracy: u16,
        automatic_advance_distance: Option<u16>,
    ) {
        let current_route_step = gen_dummy_route_step(x1, y1, x2, y2);
        let next_route_step = if has_next_step {
            Some(gen_dummy_route_step(x2, y2, x3, y3))
        } else {
            None
        };

        // Construct a user location that's slightly offset from the transition point with perfect accuracy
        let end_of_step = *current_route_step.geometry.last().unwrap();
        let user_location = UserLocation {
            coordinates: GeographicCoordinate {
                lng: end_of_step.lng + error,
                lat: end_of_step.lat + error,
            },
            horizontal_accuracy: 0.0,
            course_over_ground: None,
            timestamp: SystemTime::now(),
            speed: None
        };
        let user_location_point = Point::from(user_location);
        let distance_from_end_of_current_step = Haversine::distance(user_location_point, end_of_step.into());

        // Never advance to the next step when StepAdvanceMode is Manual
        prop_assert!(!should_advance_to_next_step(&current_route_step.get_linestring(), next_route_step.as_ref(), &user_location, StepAdvanceMode::Manual));

        // Assumes that underlying distance calculations in GeoRust are correct is correct
        prop_assert_eq!(should_advance_to_next_step(&current_route_step.get_linestring(), next_route_step.as_ref(), &user_location, StepAdvanceMode::DistanceToEndOfStep {
            distance, minimum_horizontal_accuracy
        }), distance_from_end_of_current_step <= distance.into(), "Expected that the step should advance in this case as we are closer to the end of the step than the threshold.");

        // Similar test for automatic advance on the relative line string distance mode
        if automatic_advance_distance.map_or(false, |advance_distance| {
            distance_from_end_of_current_step <= advance_distance.into()
        }) {
            prop_assert!(should_advance_to_next_step(&current_route_step.get_linestring(), next_route_step.as_ref(), &user_location, StepAdvanceMode::RelativeLineStringDistance {
                minimum_horizontal_accuracy,
                special_advance_conditions: automatic_advance_distance.map(|distance| SpecialAdvanceConditions::AdvanceAtDistanceFromEnd(distance)),
            }), "Expected that the step should advance any time that the haversine distance to the end of the step is within the automatic advance threshold.");
        }
    }

    #[test]
    fn test_end_of_step_progress(
        x1 in -180f64..180f64, y1 in -90f64..90f64,
        x2 in -180f64..180f64, y2 in -90f64..90f64,
    ) {
        let current_route_step = gen_dummy_route_step(x1, y1, x2, y2);
        let linestring = current_route_step.get_linestring();
        let end = linestring.points().last().expect("Expected at least one point");
        let progress = calculate_trip_progress(&end, &linestring, &[current_route_step]);

        prop_assert_eq!(progress.distance_to_next_maneuver, 0f64);
        prop_assert_eq!(progress.distance_remaining, 0f64);
        prop_assert_eq!(progress.duration_remaining, 0f64);
    }

    #[test]
    fn test_end_of_trip_progress_valhalla_arrival(
        x1: f64, y1: f64,
    ) {
        // This may look wrong, but it's actually how Valhalla (and presumably others)
        // represent a point geometry for the arrival step.
        let current_route_step = gen_dummy_route_step(x1, y1, x1, y1);
        let linestring = current_route_step.get_linestring();
        let end = linestring.points().last().expect("Expected at least one point");
        let progress = calculate_trip_progress(&end, &linestring, &[current_route_step]);

        prop_assert_eq!(progress.distance_to_next_maneuver, 0f64);
        prop_assert_eq!(progress.distance_remaining, 0f64);
        prop_assert_eq!(progress.duration_remaining, 0f64);
    }

    #[test]
    fn test_geometry_index_empty_linestring(
        x: f64, y: f64,
    ) {
        let index = index_of_closest_segment_origin(make_user_location(x, y), &LineString::new(vec![]));
        prop_assert_eq!(index, None);
    }

    #[test]
    fn test_geometry_index_single_coord_invalid_linestring(
        x: f64, y: f64,
    ) {
        let index = index_of_closest_segment_origin(make_user_location(x, y), &LineString::new(vec![coord! { x: x, y: y }]));
        prop_assert_eq!(index, None);
    }

    #[test]
    fn test_geometry_index_is_some_for_reasonable_linestrings(
        x in -180f64..180f64, y in -90f64..90f64,
        coords in vec(arb_coord(), 2..500)
    ) {
        let index = index_of_closest_segment_origin(make_user_location(x, y), &LineString::new(coords));

        // There are at least two points, so we have a valid segment
        prop_assert_ne!(index, None);
    }

    #[test]
    fn test_geometry_index_at_terminal_coord(
        coords in vec(arb_coord(), 2..500)
    ) {
        let last_coord = coords.last().unwrap();
        let coord_len = coords.len();
        let user_location = make_user_location(last_coord.x, last_coord.y);
        let index = index_of_closest_segment_origin(user_location, &LineString::new(coords));

        // There are at least two points, so we have a valid segment
        prop_assert_ne!(index, None);
        let index = index.unwrap();
        // We should never be able to go past the origin of the final pair
        prop_assert!(index < (coord_len - 1) as u64);
    }

    #[test]
    fn test_bearing_fuzz(coords in vec(arb_coord(), 2..500), index in 0usize..1_000usize) {
        let line = LineString::new(coords);
        let result = get_bearing_to_next_point(index, &line);
        if index < line.coords_count() - 1 {
            prop_assert!(result.is_some());
        } else {
            prop_assert!(result.is_none());
        }
    }

    #[test]
    fn test_bearing_end_of_line(coords in vec(arb_coord(), 2..500)) {
        let line = LineString::new(coords);
        prop_assert!(get_bearing_to_next_point(line.coords_count(), &line).is_none());
        prop_assert!(get_bearing_to_next_point(line.coords_count() - 1, &line).is_none());
        prop_assert!(get_bearing_to_next_point(line.coords_count() - 2, &line).is_some());
    }
}

#[cfg(test)]
mod linestring_based_tests {

    use super::*;

    static COORDS: [Coord; 5] = [
        coord!(x: 0.0, y: 0.0),
        coord!(x: 1.0, y: 1.0),
        coord!(x: 2.0, y: 2.0),
        coord!(x: 3.0, y: 3.0),
        coord!(x: 4.0, y: 4.0),
    ];

    #[test]
    fn test_geometry_index_at_point() {
        let line = LineString::new(COORDS.to_vec());

        // Exactly at a point (NB: does not advance until we move *past* the transition point
        // and are closer to the next line segment!)
        let index = index_of_closest_segment_origin(make_user_location(2.0, 2.0), &line);
        assert_eq!(index, Some(1));
    }

    #[test]
    fn test_geometry_index_near_point() {
        let line = LineString::new(COORDS.to_vec());

        // Very close to an origin point
        let index = index_of_closest_segment_origin(make_user_location(1.1, 1.1), &line);
        assert_eq!(index, Some(1));

        // Very close to the next point, but not yet "passing" to the next segment!
        let index = index_of_closest_segment_origin(make_user_location(1.99, 1.99), &line);
        assert_eq!(index, Some(1));
    }

    #[test]
    fn test_geometry_index_far_from_point() {
        let line = LineString::new(COORDS.to_vec());

        // "Before" the start
        let index = index_of_closest_segment_origin(make_user_location(-1.1, -1.1), &line);
        assert_eq!(index, Some(0));

        // "Past" the end (NB: the last index in the list of coords is 4,
        // but we can never advance past n-1)
        let index = index_of_closest_segment_origin(make_user_location(10.0, 10.0), &line);
        assert_eq!(index, Some(3));
    }
}

#[cfg(test)]
mod bearing_snapping_tests {

    use super::*;

    static COORDS: [Coord; 6] = [
        coord!(x: 0.0, y: 0.0),
        coord!(x: 1.0, y: 1.0),
        coord!(x: 2.0, y: 1.0),
        coord!(x: 2.0, y: 2.0),
        coord!(x: 2.0, y: 1.0),
        coord!(x: 1.0, y: 1.0),
    ];

    #[test]
    fn test_bearing_to_next_point() {
        let line = LineString::new(COORDS.to_vec());

        let bearing = get_bearing_to_next_point(0, &line);
        assert_eq!(
            bearing,
            Some(CourseOverGround {
                degrees: 45,
                accuracy: None
            })
        );

        let bearing = get_bearing_to_next_point(1, &line);
        assert_eq!(
            bearing,
            Some(CourseOverGround {
                degrees: 90,
                accuracy: None
            })
        );

        let bearing = get_bearing_to_next_point(2, &line);
        assert_eq!(
            bearing,
            Some(CourseOverGround {
                degrees: 0,
                accuracy: None
            })
        );

        let bearing = get_bearing_to_next_point(3, &line);
        assert_eq!(
            bearing,
            Some(CourseOverGround {
                degrees: 180,
                accuracy: None
            })
        );

        let bearing = get_bearing_to_next_point(4, &line);
        assert_eq!(
            bearing,
            Some(CourseOverGround {
                degrees: 270,
                accuracy: None
            })
        );

        // At the end
        let bearing = get_bearing_to_next_point(5, &line);
        assert_eq!(bearing, None);
    }

    #[test]
    fn test_apply_snapped_course() {
        let line = LineString::new(COORDS.to_vec());

        // The value of the coordinates does not actually matter;
        // we are testing the course snapping
        let user_location = make_user_location(5.0, 1.0);

        // Apply a course to a user location
        let updated_location = apply_snapped_course(user_location, Some(1), &line);

        assert_eq!(
            updated_location.course_over_ground,
            Some(CourseOverGround {
                degrees: 90,
                accuracy: None
            })
        );
    }
}

#[cfg(test)]
mod adaptive_distance_tests {
    use super::*;
    use std::f64::EPSILON;

    // -------------- Basic Functionality Tests --------------

    /// Tests the initial behavior of the algorithm (defaults to Haversine)
    #[test]
    fn test_basic_functionality() {
        let calc = DistanceCalculation::default_adaptive();
        let p1 = Point::new(0.0, 0.0);
        let p2 = Point::new(0.1, 0.0);
        
        let (distance, new_calc) = calc.calculate_distance(p1, p2);
        
        if let DistanceCalculation::Adaptive { current_algorithm, .. } = new_calc {
            assert_eq!(current_algorithm, AdaptiveAlgorithm::Haversine);
            assert!((distance - Haversine::distance(p1, p2)).abs() < EPSILON);
        } else {
            panic!("Expected Adaptive variant");
        }
    }

    /// Tests evidence decay mechanism
    #[test]
    fn test_evidence_decay() {
        let mut calc = DistanceCalculation::Adaptive {
            current_algorithm: AdaptiveAlgorithm::Haversine,
            evidence: 0.7,
            switch_threshold: 1.0,
            evidence_decay: 0.2,
        };
        
        let neutral_points = [(0.0, 30.0), (0.01, 30.0), (0.02, 30.0)];
        let initial_evidence = if let DistanceCalculation::Adaptive { evidence, .. } = calc { evidence } else { 0.0 };
        
        for i in 0..neutral_points.len()-1 {
            let (_, new_calc) = calc.calculate_distance(
                Point::new(neutral_points[i].0, neutral_points[i].1),
                Point::new(neutral_points[i+1].0, neutral_points[i+1].1)
            );
            calc = new_calc;
            
            if let DistanceCalculation::Adaptive { evidence, .. } = &calc {
                println!("Decay test - Iteration {}: Evidence: {:.4}", i, evidence);
            }
        }
        
        if let DistanceCalculation::Adaptive { evidence, .. } = calc {
            assert!(evidence < initial_evidence, "Evidence should decay when conditions are neutral");
        }
    }

    // -------------- Algorithm Switching Tests --------------

    /// Tests switching to Geodesic algorithm at high latitudes
    #[test]
    fn test_switching_to_geodesic_at_high_latitudes() {
        let mut calc = DistanceCalculation::Adaptive {
            current_algorithm: AdaptiveAlgorithm::Haversine,
            evidence: 0.0,
            switch_threshold: 0.8,
            evidence_decay: 0.05,
        };
        
        let polar_points = [(0.0, 80.0), (0.5, 80.0), (1.0, 80.0), (1.5, 80.0)];
        let mut switched = false;
        
        for i in 0..polar_points.len()-1 {
            let (_, new_calc) = calc.calculate_distance(
                Point::new(polar_points[i].0, polar_points[i].1),
                Point::new(polar_points[i+1].0, polar_points[i+1].1)
            );
            
            if let DistanceCalculation::Adaptive { current_algorithm, evidence, .. } = &new_calc {
                println!("Polar test - Point {}: Algorithm: {:?}, Evidence: {:.4}", i, current_algorithm, evidence);
                
                if *current_algorithm == AdaptiveAlgorithm::Geodesic {
                    if let DistanceCalculation::Adaptive { current_algorithm: prev_algorithm, .. } = &calc {
                        // Only check evidence reset if we JUST switched
                        if *prev_algorithm == AdaptiveAlgorithm::Haversine {
                            switched = true;
                            assert_eq!(*evidence, 0.0, "Evidence should reset after switching");
                        }
                    }
                }
            }
            
            calc = new_calc;
        }
        
        assert!(switched, "Failed to switch to Geodesic in high latitudes");
    }

    /// Tests switching back to Haversine at low latitudes
    #[test]
    fn test_switching_back_to_haversine() {
        let mut calc = DistanceCalculation::Adaptive {
            current_algorithm: AdaptiveAlgorithm::Geodesic,
            evidence: -0.5,  // Start with evidence toward Haversine
            switch_threshold: 0.8,
            evidence_decay: 0.05,
        };
        
        let equatorial_points = [(0.0, 0.0), (0.01, 0.01), (0.02, 0.0), (0.03, -0.01)];
        
        let mut switched_to_haversine = false;
        
        for i in 0..equatorial_points.len()-1 {
            let (_, new_calc) = calc.calculate_distance(
                Point::new(equatorial_points[i].0, equatorial_points[i].1),
                Point::new(equatorial_points[i+1].0, equatorial_points[i+1].1)
            );
            calc = new_calc;
            
            if let DistanceCalculation::Adaptive { current_algorithm, evidence, .. } = &calc {
                println!("Switching back test - Point {}: Algorithm: {:?}, Evidence: {:.4}", i, current_algorithm, evidence);
                
                if *current_algorithm == AdaptiveAlgorithm::Haversine {
                    switched_to_haversine = true;
                    assert_eq!(*evidence, 0.0, "Evidence should reset after switching back");
                    break;
                }
            }
        }
        
        assert!(switched_to_haversine, "Failed to switch back to Haversine for favorable conditions");
    }

    /// Tests threshold sensitivity
    #[test]
    fn test_threshold_sensitivity() {
        let thresholds = [0.5, 1.0, 2.0];
        let p1 = Point::new(0.0, 75.0);
        let p2 = Point::new(0.1, 75.0);
        
        println!("\nTesting threshold sensitivity:");
        let mut iterations_for_thresholds = Vec::new();
        
        for threshold in thresholds {
            let mut calc = DistanceCalculation::Adaptive {
                current_algorithm: AdaptiveAlgorithm::Haversine,
                evidence: 0.0,
                switch_threshold: threshold,
                evidence_decay: 0.1,
            };
            
            let mut iterations_to_switch = 0;
            let mut switched = false;
            
            for i in 1..=20 {
                let (_, new_calc) = calc.calculate_distance(p1, p2);
                calc = new_calc;
                
                if let DistanceCalculation::Adaptive { current_algorithm, .. } = &calc {
                    if *current_algorithm == AdaptiveAlgorithm::Geodesic {
                        switched = true;
                        iterations_to_switch = i;
                        break;
                    }
                }
            }
            
            println!("Threshold {}: {} iterations to switch, switched: {}", threshold, iterations_to_switch, switched);
            
            if switched {
                iterations_for_thresholds.push((threshold, iterations_to_switch));
            }
        }
        
        // Higher thresholds should require more iterations to switch
        for i in 0..iterations_for_thresholds.len()-1 {
            let (threshold1, iterations1) = iterations_for_thresholds[i];
            let (threshold2, iterations2) = iterations_for_thresholds[i+1];
            
            if threshold1 < threshold2 {
                assert!(iterations1 <= iterations2, "Higher threshold {} should require more iterations than threshold {}", threshold2, threshold1);
            }
        }
    }
    
    /// Tests multiple algorithm switches in a realistic journey
    #[test]
    fn test_multiple_algorithm_switches() {
        let mut calc = DistanceCalculation::Adaptive {
            current_algorithm: AdaptiveAlgorithm::Haversine,
            evidence: 0.0,
            switch_threshold: 0.7,
            evidence_decay: 0.1,
        };
        
        // Journey with gradual latitude changes
        let journey_points = [
            // Equator (Haversine)
            (0.0, 0.0), (0.1, 0.0), 
            // Moving north (gradually building evidence for Geodesic)
            (0.3, 10.0), (0.5, 30.0), (0.7, 50.0), (0.9, 70.0), (1.0, 80.0),
            // Stay at high latitude (should use Geodesic)
            (1.2, 85.0), (1.3, 85.0),
            // Back to equator (should build evidence for Haversine)
            (1.5, 70.0), (1.7, 50.0), (1.9, 30.0), (2.1, 10.0), (2.2, 0.0),
            // To southern high latitudes (should build evidence for Geodesic again)
            (2.5, -10.0), (2.7, -30.0), (2.9, -50.0), (3.1, -70.0), (3.2, -80.0),
            // Back to equator (should switch to Haversine again)
            (3.5, -50.0), (3.7, -30.0), (3.9, -10.0), (4.0, 0.0),
        ];
        
        println!("\nTesting multiple algorithm switches:");
        
        let mut current_algorithm = AdaptiveAlgorithm::Haversine;
        let mut switch_points = Vec::new();
        
        for i in 0..journey_points.len()-1 {
            let (_, new_calc) = calc.calculate_distance(
                Point::new(journey_points[i].0, journey_points[i].1),
                Point::new(journey_points[i+1].0, journey_points[i+1].1)
            );
            calc = new_calc;
            
            if let DistanceCalculation::Adaptive { current_algorithm: alg, evidence, .. } = &calc {
                if *alg != current_algorithm {
                    switch_points.push((i, current_algorithm, *alg));
                    println!("  SWITCHED FROM {:?} TO {:?} at point ({:.1}°,{:.1}°)", 
                            current_algorithm, alg, journey_points[i+1].0, journey_points[i+1].1);
                    current_algorithm = *alg;
                }
            }
        }
        
        // Should have at least 4 switches (H→G→H→G→H pattern)
        assert!(switch_points.len() >= 3, "Expected at least 4 algorithm switches, got {}", switch_points.len());
        
        if switch_points.len() >= 4 {
            // Verify the pattern of switches
            assert_eq!(switch_points[0].1, AdaptiveAlgorithm::Haversine);
            assert_eq!(switch_points[0].2, AdaptiveAlgorithm::Geodesic);
            
            assert_eq!(switch_points[1].1, AdaptiveAlgorithm::Geodesic);
            assert_eq!(switch_points[1].2, AdaptiveAlgorithm::Haversine);
            
            assert_eq!(switch_points[2].1, AdaptiveAlgorithm::Haversine);
            assert_eq!(switch_points[2].2, AdaptiveAlgorithm::Geodesic);
            
            assert_eq!(switch_points[3].1, AdaptiveAlgorithm::Geodesic);
            assert_eq!(switch_points[3].2, AdaptiveAlgorithm::Haversine);
        }
    }

    // -------------- Edge Cases Tests --------------

    /// Tests near the poles where Haversine has the most error
    #[test]
    fn test_near_pole_navigation() {
        let mut calc = DistanceCalculation::default_adaptive();
        
        let polar_points = [
            (0.0, 89.9),    // Near North Pole
            (90.0, 89.9),   // 90° longitude shift at same latitude
            (180.0, 89.9),  // 180° longitude shift at same latitude
        ];
        
        println!("\nTesting near-pole navigation:");
        
        for i in 0..polar_points.len()-1 {
            let (distance, new_calc) = calc.calculate_distance(
                Point::new(polar_points[i].0, polar_points[i].1),
                Point::new(polar_points[i+1].0, polar_points[i+1].1)
            );
            calc = new_calc;
            
            if let DistanceCalculation::Adaptive { current_algorithm, .. } = &calc {
                println!("Pole test - Distance: {:.2}km, Algorithm: {:?}", distance/1000.0, current_algorithm);
                
                // Should switch to Geodesic quickly at high latitudes
                if i > 0 {
                    assert_eq!(*current_algorithm, AdaptiveAlgorithm::Geodesic, "Should use Geodesic algorithm near poles");
                }
            }
        }
    }

    /// Tests international date line crossing
    #[test]
    fn test_international_date_line_crossing() {
        let mut calc = DistanceCalculation::default_adaptive();
        
        // Points crossing the international date line
        let points = [(179.9, 45.0), (-179.9, 45.0), (-179.0, 45.0)];
        
        for i in 0..points.len()-1 {
            let (distance, new_calc) = calc.calculate_distance(
                Point::new(points[i].0, points[i].1),
                Point::new(points[i+1].0, points[i].1)
            );
            calc = new_calc;
            
            // The distance should be reasonable (not going around the world)
            assert!(distance < 400000.0, "Distance across date line should be calculated correctly");
        }
    }

    /// Tests extreme situations that should force algorithm switching
    #[test]
    fn test_extreme_edge_cases() {
        let mut calc = DistanceCalculation::default_adaptive();
        
        // Points designed to create extreme conditions
        let extreme_points = [
            (0.0, 0.0),       // Equator
            (0.0, 89.5),      // Near North Pole
            (180.0, 89.5),    // Date line at high latitude
            (180.0, -89.5),   // South Pole region
        ];
        
        let mut switched_to_geodesic = false;
        
        for i in 0..extreme_points.len()-1 {
            let (_, new_calc) = calc.calculate_distance(
                Point::new(extreme_points[i].0, extreme_points[i].1),
                Point::new(extreme_points[i+1].0, extreme_points[i+1].1)
            );
            calc = new_calc;
            
            if let DistanceCalculation::Adaptive { current_algorithm, .. } = &calc {
                if *current_algorithm == AdaptiveAlgorithm::Geodesic {
                    switched_to_geodesic = true;
                }
            }
        }
        
        assert!(switched_to_geodesic, "Should switch to Geodesic under extreme conditions");
    }
    
    /// Tests invalid coordinate handling
    #[test]
    fn test_invalid_coordinates() {
        let calc = DistanceCalculation::default_adaptive();
        
        // Test various invalid coordinates
        let invalid_cases = [
            (Point::new(f64::NAN, 0.0), Point::new(0.0, 0.0)),
            (Point::new(0.0, f64::NAN), Point::new(0.0, 0.0)),
            (Point::new(f64::INFINITY, 0.0), Point::new(0.0, 0.0)),
        ];
        
        for (p1, p2) in &invalid_cases {
            let (distance, _) = calc.calculate_distance(*p1, *p2);
            assert_eq!(distance, 0.0, "Should return 0.0 for invalid coordinates");
        }
    }

    /// Tests tiny coordinate changes
    #[test]
    fn test_tiny_coordinate_changes() {
        let calc = DistanceCalculation::default_adaptive();
        
        let p1 = Point::new(0.0, 0.0);
        let p2 = Point::new(0.00000001, 0.00000001); // Extremely close points
        
        let (distance, _) = calc.calculate_distance(p1, p2);
        
        // Distance should be very small but not zero or NaN
        assert!(distance > 0.0 && distance < 1.0, "Distance should be small but valid");
        assert!(!distance.is_nan(), "Distance should not be NaN");
    }

    // -------------- Performance/Stability Tests --------------

    /// Tests prevention of algorithm thrashing in rapidly changing conditions
    #[test]
    fn test_algorithm_stability_in_zigzag_pattern() {
        let mut calc = DistanceCalculation::default_adaptive();
        
        // Create a zigzag pattern that alternates between high and low latitudes
        let zigzag_points = [
            (0.0, 10.0),  // Low latitude
            (0.1, 70.0),  // High latitude
            (0.2, 15.0),  // Low latitude
            (0.3, 75.0),  // High latitude
            (0.4, 20.0),  // Low latitude
            (0.5, 80.0),  // High latitude
        ];
        
        println!("\nTesting zigzag route pattern:");
        
        let mut algorithm_switches = 0;
        let mut current_algorithm = AdaptiveAlgorithm::Haversine;
        
        for i in 0..zigzag_points.len()-1 {
            let (_, new_calc) = calc.calculate_distance(
                Point::new(zigzag_points[i].0, zigzag_points[i].1),
                Point::new(zigzag_points[i+1].0, zigzag_points[i+1].1)
            );
            calc = new_calc;
            
            if let DistanceCalculation::Adaptive { current_algorithm: alg, .. } = &calc {
                if *alg != current_algorithm {
                    algorithm_switches += 1;
                    current_algorithm = *alg;
                }
            }
        }
        
        // Should not thrash between algorithms
        assert!(algorithm_switches <= 2, "Algorithm should not thrash between modes (had {} switches)", algorithm_switches);
    }
}

// TODO: Other unit tests
// - Under and over distance accuracy thresholds
// - Equator and extreme latitude
