// Open Set: This is where you store nodes that are yet to be evaluated.
//      A binary heap (BinaryHeap in Rust's standard library) is a good choice for the open set
//      as it allows you to efficiently get the node with the lowest f value, which is essential for A*.
// Closed Set: This set contains nodes that have already been evaluated.
//      A HashSet is typically used for the closed set to allow for quick lookups to check if
//      a node has already been processed.

//! A* path planner

use dubins_paths::{DubinsPath, PosRot};
use geo;
use geo::algorithm::intersects::Intersects;
use geo::Contains;
use geo_types::coord;
use geographiclib_rs::{DirectGeodesic, Geodesic, InverseGeodesic};
use std::collections::BinaryHeap;
use std::collections::HashMap;
use std::f64::consts::PI;
use std::rc::Rc;

use crate::mav_link_plan;
mod planning_waypoints;
use planning_waypoints::{GeospatialPose, Node};

/// Data for the A* planner.
pub struct AStarPlanner {
    start_pose: GeospatialPose, // Waypoint where take off sequence ends
    goal_pose: GeospatialPose,  // Where do you want to fly
    end_pose: GeospatialPose,   // Waypoint where landing sequence starts
    optimal_path_from_start_to_goal: Vec<Rc<Node>>,
    optimal_path_from_goal_to_end: Vec<Rc<Node>>,
    geo_fences_polygon: geo::MultiLineString,
    geo_fences_circles: Vec<geo::Point>,
}

impl AStarPlanner {
    /// Builds A* Planner struct
    ///
    /// # Arguments
    /// * `start_location` - Longitude and latitude of point where take off sequence ends as a geo::Point<f64>.
    /// * `start_heading` - Heading of point where take off sequence ends as an f64.
    /// * `goal_location` - Longitude and latitude of point where you want to go.
    /// * `end_location` - Longitude and latitude of point where landing sequence starts as a geo::Point<f64>.
    /// * `end_heading` - Heading of point where landing sequence starts as an f64.
    ///
    /// # Returns
    /// AStarPlanner struct in a Result<>.
    pub fn new(
        start_location: geo::Point,
        start_heading: f64,
        goal_location: geo::Point,
        end_location: geo::Point,
        end_heading: f64,
    ) -> Result<Self, String> {
        let start_pose = GeospatialPose {
            position: start_location,
            height: 120.0,
            heading: start_heading,
        };
        let goal_pose = GeospatialPose {
            position: goal_location,
            height: 120.0,
            heading: 0.0,
        };
        let end_pose = GeospatialPose {
            position: end_location,
            height: 120.0,
            heading: end_heading,
        };

        let geo_fence_border = geo::LineString::new(vec![
            geo::coord! {x:52.28295542244744, y: 6.8565871319299845},
            geo::coord! {x:52.285431953652584, y: 6.86156240560706},
            geo::coord! {x:52.2896098479409, y: 6.8688319693996505},
            geo::coord! {x:52.29227492433401, y: 6.875640979066702},
            geo::coord! {x:52.2937453230195, y: 6.883632543793112},
            geo::coord! {x:52.29268600915777, y: 6.888311871426254},
            geo::coord! {x:52.294277072434966, y: 6.889956775263698},
            geo::coord! {x:52.293939049265504, y: 6.896312607978928},
            geo::coord! {x:52.29294882408129, y: 6.902284711120359},
            geo::coord! {x:52.28900135729954, y: 6.917255476279564},
            geo::coord! {x:52.2714465433939, y: 6.876881353338064},
            geo::coord! {x:52.27574362849022, y: 6.869559476902992},
            geo::coord! {x:52.27691573983945, y: 6.868882808809673},
            geo::coord! {x:52.27793930785583, y: 6.867776234580788},
            geo::coord! {x:52.27856267193794, y: 6.865940640293019},
            geo::coord! {x:52.27956352880396, y: 6.862960555652251},
            geo::coord! {x:52.281442698183156, y: 6.8598604985957365},
            // repeat first point to close the loop
            geo::coord! {x:52.28295542244744, y: 6.8565871319299845},
        ]);
        let geo_fence_border_polygon = geo::Polygon::new(geo_fence_border.clone(), vec![]);

        // The start and goal position should be inside the geo_fence
        if !geo_fence_border_polygon.contains(&start_pose.position) {
            return Err(String::from("Ongeldige drone positie."));
        }
        if !geo_fence_border_polygon.contains(&goal_pose.position) {
            return Err(String::from("Ongeldige doel positie."));
        }

        let a_star_planner = AStarPlanner {
            start_pose: start_pose,
            goal_pose: goal_pose,
            end_pose: end_pose,
            optimal_path_from_start_to_goal: Vec::new(),
            optimal_path_from_goal_to_end: Vec::new(),
            geo_fences_polygon: geo::MultiLineString::new(vec![]),
            geo_fences_circles: vec![],
        };

        Ok(a_star_planner)
    }

    /// Read a MavLink GeoFence structure and translate it to geo::Points and geo::LineStrings
    /// for use by the A* Planner.
    pub fn add_geo_fences(&mut self, geo_fences: &mav_link_plan::GeoFence) {
        let mut geo_fence_polygons_vector = vec![];
        for geo_fence_polygon in geo_fences.polygons.iter() {
            let mut polygon_vector = vec![];
            for point in geo_fence_polygon.polygon.iter() {
                polygon_vector.push(coord! {x: point[0], y: point[1]});
            }
            polygon_vector.push(polygon_vector[0]);
            geo_fence_polygons_vector.push(geo::LineString::new(polygon_vector));
        }
        self.geo_fences_polygon = geo::MultiLineString::new(geo_fence_polygons_vector);

        for geo_fence_circle in geo_fences.circles.iter() {
            self.geo_fences_circles.push(geo::Point::new(
                geo_fence_circle.circle.center[0],
                geo_fence_circle.circle.center[1],
            ));
        }
    }

    /// Calculate path from position and orientation of begin_pose to the position of the end_pose.
    /// This does not take the end orientation into account.
    fn calculate_path(
        &self,
        begin_pose: &GeospatialPose,
        end_pose: &GeospatialPose,
    ) -> Vec<Rc<Node>> {
        let geod = Geodesic::wgs84();
        let mut open_set: BinaryHeap<Rc<Node>> = BinaryHeap::new();
        let mut closed_set: HashMap<GeospatialPose, Rc<Node>> = HashMap::new();
        let mut optimal_path = Vec::new();
        let distance_increment = 210.0;

        let cost_to_goal = calculate_h(begin_pose, end_pose);
        let begin_node = Rc::new(planning_waypoints::Node {
            pose: begin_pose.clone(),
            traveled_distance: 0.0,
            steering: 0.0,
            steering_integral: 0.0,
            g: 0.0,
            h: cost_to_goal,
            f: cost_to_goal + 0.0,
            parent: None,
        });

        open_set.push(begin_node);

        let mut path_not_found: bool = true;
        let mut points_in_reach_of_goal: BinaryHeap<Rc<Node>> = BinaryHeap::new();
        while path_not_found {
            // Pull top position from Open Set
            let parent = open_set.pop().unwrap();

            // calculate new positions
            let mut next_poses = parent.pose.calculate_next_poses(distance_increment);
            // Check validity of new positions
            // Check if new position are in Closed Set (already treated)
            next_poses.retain(|&pose| {
                pose.is_valid(
                    &parent.pose,
                    &self.geo_fences_polygon,
                    &self.geo_fences_circles,
                ) && !closed_set.contains_key(&pose)
            });

            // Calculate node information of new position
            for pose in next_poses.iter() {
                let steering = pose.heading - parent.pose.heading;
                let steering_integral = parent.steering_integral + steering.abs();

                let cost_so_far = parent.g + calculate_delta_g(&pose, &parent, distance_increment);
                let cost_to_goal = calculate_h(&pose, end_pose);

                let node = Rc::new(Node {
                    pose: *pose,
                    traveled_distance: parent.traveled_distance + distance_increment,
                    steering: steering,
                    steering_integral: steering_integral,
                    g: cost_so_far,
                    h: cost_to_goal,
                    f: cost_so_far + cost_to_goal + 0.0001 * steering_integral * steering_integral,
                    parent: Some(Rc::clone(&parent)),
                });

                // Check if new positions is goal
                let (distance, _, _, _) = geod.inverse(
                    node.pose.position.x(),
                    node.pose.position.y(),
                    end_pose.position.x(),
                    end_pose.position.y(),
                );
                if distance < distance_increment {
                    path_not_found = false;
                    points_in_reach_of_goal.push(Rc::clone(&node));
                    closed_set.insert(node.pose, node);
                } else {
                    // Push new positions to Open Set
                    open_set.push(node);
                }
            }

            // Push top position to Closed Set
            closed_set.insert(parent.pose, parent);
        }
        optimal_path.push(Rc::clone(&points_in_reach_of_goal.pop().unwrap()));

        // Fill the vector with waypoints from goal pose to start pose.
        loop {
            let node = closed_set
                .get(&optimal_path[optimal_path.len() - 1].pose)
                .unwrap();
            match &node.parent {
                Some(value) => optimal_path.push(Rc::clone(&value)),
                None => break,
            }
        }

        optimal_path
    }

    pub fn get_optimal_path_to_goal(&mut self) -> Vec<geo::Point> {
        self.optimal_path_from_start_to_goal =
            self.calculate_path(&self.start_pose, &self.goal_pose);

        let mut optimal_path_from_start_to_goal: Vec<geo::Point> = Vec::new();

        if self.optimal_path_from_start_to_goal.len() > 2 {
            for point in self.optimal_path_from_start_to_goal[1..].iter().rev() {
                optimal_path_from_start_to_goal.push(point.pose.position);
            }
        }

        optimal_path_from_start_to_goal
    }

    pub fn get_optimal_path_from_goal(&mut self) -> Vec<geo::Point> {
        let mut node_inverse_heading = self.end_pose.clone();
        node_inverse_heading.heading += 180.0;
        self.optimal_path_from_goal_to_end =
            self.calculate_path(&node_inverse_heading, &self.goal_pose);

        let mut optimal_path_from_goal_to_end: Vec<geo::Point> = Vec::new();

        for point in self.optimal_path_from_goal_to_end.iter() {
            optimal_path_from_goal_to_end.push(point.pose.position);
        }

        optimal_path_from_goal_to_end
    }
}

impl GeospatialPose {
    fn calculate_next_poses(&self, distance_increment: f64) -> Vec<GeospatialPose> {
        let mut next_poses = Vec::new();

        // turn 90 degree left
        next_poses.push(self.calculate_next_pose(-90.0, distance_increment));
        // turn 45 degree left
        next_poses.push(self.calculate_next_pose(-45.0, distance_increment));
        // turn 22.5 degree left
        next_poses.push(self.calculate_next_pose(-22.5, distance_increment));
        // turn 11.25 degree left
        next_poses.push(self.calculate_next_pose(-11.25, distance_increment));
        // straight ahead
        next_poses.push(self.calculate_next_pose(0.0, distance_increment));
        // turn 11.25 degrees right
        next_poses.push(self.calculate_next_pose(11.25, distance_increment));
        // turn 22.5 degrees right
        next_poses.push(self.calculate_next_pose(22.5, distance_increment));
        // turn 45 degrees right
        next_poses.push(self.calculate_next_pose(45.0, distance_increment));
        // turn 90 degrees right
        next_poses.push(self.calculate_next_pose(90.0, distance_increment));

        return next_poses;
    }

    fn calculate_next_pose(&self, delta_heading: f64, distance_increment: f64) -> Self {
        let geod = Geodesic::wgs84();

        if delta_heading == 0.0 {
            let (lat, lon, _) = geod.direct(
                self.position.x(),
                self.position.y(),
                self.heading,
                distance_increment,
            );
            let point_straight_ahead = geo::Point::new(lat, lon);
            GeospatialPose {
                position: point_straight_ahead,
                heading: self.heading,
                height: 120.0,
            }
        } else {
            let delta_movement_direction = delta_heading / 2.0;

            let new_heading = self.calculate_new_heading(delta_heading);
            let movement_direction = self.calculate_new_heading(delta_movement_direction);
            let chord_length = chord_length(distance_increment, delta_heading);
            let (lat, lon, _) = geod.direct(
                self.position.x(),
                self.position.y(),
                movement_direction,
                chord_length,
            );
            let point_left_90 = geo::Point::new(lat, lon);

            GeospatialPose {
                position: point_left_90,
                heading: new_heading,
                height: 120.0,
            }
        }
    }

    /// Calculates new heading and put it in 0-360 range.
    fn calculate_new_heading(&self, angle: f64) -> f64 {
        let mut new_heading = self.heading + angle;
        if new_heading < 0.0 {
            new_heading += 360.0;
        } else if new_heading >= 360.0 {
            new_heading -= 360.0;
        }

        new_heading
    }

    fn is_valid(
        &self,
        parent_pose: &GeospatialPose,
        geo_fence_polygon: &geo::MultiLineString,
        geo_fence_circle: &Vec<geo::Point>,
    ) -> bool {
        let drone_path = geo::Line::new(
            geo::coord! {x:parent_pose.position.x(), y:parent_pose.position.y()},
            geo::coord! {x:self.position.x(), y:self.position.y()},
        );

        if geo_fence_polygon.intersects(&drone_path) {
            false
        } else {
            for point in geo_fence_circle.iter() {
                if circle_line_intersect(&point, 140.0, &drone_path) {
                    return false;
                }
            }
            true
        }
    }
}

/// Calculates the chord length of a circle given the arc length and central angle in degrees.
///
/// The function requires the central angle to be within [-90, 90] degrees to avoid unrealistic scenarios.
/// It calculates the chord length using the formula derived from the relationship between the arc length,
/// the radius of the circle, and the central angle. The angle is first converted from degrees to radians
/// for the calculation.
///
/// Parameters:
/// - `arc_length`: The length of the arc subtended by the chord, a positive `f64`.
/// - `angle_degrees`: The central angle subtending the arc, in degrees, must be within [-90, 90].
///
/// Returns:
/// - The length of the chord as a `f64`.
///
/// Panics:
/// - If `angle_degrees` is not within [-90, 90], indicating an unrealistic heading change.
///
/// Example:
/// Assuming an arc length of 100 units and a central angle of 60 degrees, the function will calculate
/// the corresponding chord length based on these inputs.
fn chord_length(arc_length: f64, angle_degrees: f64) -> f64 {
    if angle_degrees > 90.0 || angle_degrees < -90.0 {
        panic!("Heading change out of range!");
    }
    // Convert angle from degrees to radians
    let angle_radians = angle_degrees.abs() * (PI / 180.0);

    // Calculate the radius of the circle
    let radius = arc_length / angle_radians;

    // Calculate and return the chord length
    2.0 * radius * (angle_radians / 2.0).sin()
}

/// Calculate cost so far
fn calculate_delta_g(
    child_pose: &GeospatialPose,
    parent_node: &Node,
    distance_increment: f64,
) -> f64 {
    let steering = heading_difference(child_pose.heading, parent_node.pose.heading);
    cost_function(distance_increment, steering)
}

/// Calculate cost to goal
fn calculate_h(pose: &GeospatialPose, goal_pose: &GeospatialPose) -> f64 {
    let geod = Geodesic::wgs84();
    let (lat1, lon1) = (pose.position.x(), pose.position.y()); // Start position
    let (lat2, lon2) = (goal_pose.position.x(), goal_pose.position.y()); // Goal Position
    let (s12, az1, _, _) = geod.inverse(lat1, lon1, lat2, lon2);

    let azimuth: f64;
    if az1 < 0.0 {
        azimuth = 360.0 - az1.abs();
    } else {
        azimuth = az1;
    }

    let new_heading = heading_difference(pose.heading, azimuth);
    let new_heading_rad = new_heading * 2.0 * PI / 360.0;

    let distance: f64;
    if s12 > 280.0 {
        distance = simplified_dubins_path(new_heading_rad, s12);
    } else {
        let start_position: PosRot = PosRot::from_f32(0.0, 0.0, new_heading_rad as f32);
        let goal_position: PosRot = PosRot::from_f32(s12 as f32, 0.0, 0.0);
        let shortest_path_possible =
            DubinsPath::shortest_from(start_position, goal_position, 140.0).unwrap();
        distance = shortest_path_possible.length() as f64;
    }

    cost_function(distance, new_heading)
}

/// Calculate length of Dubins Path for Left Turn - Straight
///
/// The Dubins Paths algoritm from the Rust Crate has to have an end orientation.
/// Having an end orientation results is non-optimal paths for the drone as we don't care
/// about the end position of the drone.
/// This function is a simplified version of the Dubins Path algorithm
/// * It does only Left Turn - Straight.
/// * It does not care about the end position.
/// * Start position is [0.0, 0.0, theta]
/// * End position is [0.0, distance, None]
///
/// Inputs
/// * theta: orientation with respect to line from start to end position.
/// * distance: total distance from start to end position.
fn simplified_dubins_path(theta: f64, distance: f64) -> f64 {
    if theta < 0.0 || theta > 180.0 {
        panic!("Angle is too big or too small! {}", theta);
    }

    let radius = 140.0;
    let distance_y = radius * (1.0 - theta.cos());

    let gamma = PI / 2.0 - (distance_y / distance).acos();
    let arc_length = (2.0 * PI * radius) * ((gamma + theta) / (2.0 * PI));

    let tussen_calc = distance_y / gamma.cos();
    let straight_length = (distance * distance - tussen_calc * tussen_calc).sqrt();

    arc_length + straight_length
}

/// Calculates the smallest difference between two headings.
///
/// This function determines the minimum angular difference between two headings,
/// taking into account the circular nature of bearings. Bearings are assumed to be
/// in degrees, with valid values ranging from 0 to 360 degrees. The function returns
/// the smallest difference in degrees, which will always be between 0 and 180 degrees,
/// inclusive. This represents the shortest way to turn from one heading to another.
///
/// Parameters:
/// - `heading1`: The first heading in degrees. Valid values are from 0 to 360.
/// - `heading2`: The second heading in degrees. Valid values are from 0 to 360.
///
/// Returns:
/// - A `f64` representing the smallest angular difference between the two headings in degrees.
///
/// Examples:
/// - If `heading1` is 350 degrees and `heading2` is 10 degrees, the function will return 20 degrees,
///   since turning 20 degrees from either heading will align them.
fn heading_difference(heading1: f64, heading2: f64) -> f64 {
    let diff = (heading1 - heading2).abs();
    if diff > 180.0 {
        360.0 - diff
    } else {
        diff
    }
}

/// Checks if a circle and a line (starting from (0,0) and ending at line.end) intersect.
///
/// # Arguments
/// * `circle_center` - The center of the circle as a `geo::Point<f64>``.
/// * `radius` - The radius of the circle as an f64.
/// * `line` - The line as a `geo::Line<f64>`.
///
/// # Returns
/// True if the line segment intersects the circle, false otherwise.
fn circle_line_intersect(
    circle_center: &geo::Point<f64>,
    radius: f64,
    line: &geo::Line<f64>,
) -> bool {
    let (x1, y1) = transform_geo_to_cartesian_coordinates(&circle_center, &line.start.into());
    let (x2, y2) = transform_geo_to_cartesian_coordinates(&circle_center, &line.end.into());

    let dx = x2 - x1;
    let dy = y2 - y1;

    // Coefficients for the quadratic equation Ax^2 + Bx + C = 0
    let a = dx * dx + dy * dy;
    let b = 2.0 * (x1 * dx + y1 * dy);
    let c = x1 * x1 + y1 * y1 - radius * radius;

    // Discriminant
    let discriminant = b * b - 4.0 * a * c;

    if discriminant < 0.0 {
        // No real solutions, the line does not intersect the circle
        return false;
    }

    // Calculate the t values for potential intersection points
    let t1 = (-b - discriminant.sqrt()) / (2.0 * a);
    let t2 = (-b + discriminant.sqrt()) / (2.0 * a);

    // Check if either t value is within the line segment (0 <= t <= 1)
    (t1 >= 0.0 && t1 <= 1.0) || (t2 >= 0.0 && t2 <= 1.0)
}

/// Converts geographical points to Cartesian coordinates based on a specified origin using WGS84 geodesic calculations.
///
/// Transforms geographical coordinates (latitude, longitude) into Cartesian coordinates (x, y)
/// relative to a given origin point. This is useful for representing the displacement from the
/// origin to another point on the Earth's surface in a 2D plane.
///
/// Parameters:
/// - `origin`: Reference to a `geo::Point<f64>` representing the origin point in geographical coordinates.
/// - `other_point`: Reference to a `geo::Point<f64>` representing the point to transform into Cartesian coordinates.
///
/// Returns:
/// - A tuple `(f64, f64)` representing the Cartesian coordinates (x, y) of `other_point` relative to `origin`.
///
/// Note:
/// The function uses the WGS84 model for Earth to calculate distances and bearings.
fn transform_geo_to_cartesian_coordinates(
    origin: &geo::Point<f64>,
    other_point: &geo::Point<f64>,
) -> (f64, f64) {
    let geod = Geodesic::wgs84();

    let (s12, az1, _, _) = geod.inverse(origin.x(), origin.y(), other_point.x(), other_point.y());

    let az1_pi = az1 * (PI / 180.0);

    (s12 * az1_pi.sin(), s12 * az1_pi.cos())
}

fn cost_function(distance: f64, steering: f64) -> f64 {
    // // Minimize distance. Get's the shortest path possible. Downside is that it creates a lot of nodes because
    // // all steering is allowed.
    // // distance only
    // distance

    // Minimize distance plus steering. Steering is penalized on a per node basis.
    // distance + steering
    let steering_weight = 10.0;
    distance + steering_weight * steering.abs()

    // // Minimize distance plus steering squared. Steering is penalized squared on a per node basis. This means
    // // that if we have to steer 90 degrees to get to our goal, taking a wide turn over multiple nodes if favored over
    // // a tight turn.
    // // distance + steering^2
    // let steering_weight = 4.0;
    // distance + steering_weight * steering * steering
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_heading_difference() {
        assert_eq!(heading_difference(0.0, 270.0), 90.0);
    }

    #[test]
    fn test_circle_line_intersect() {
        let circle_center = geo::Point::new(52.288131372606706, 6.875354239578542);
        let radius = 100.0;
        let line = geo::Line::new(
            geo::coord! {x:52.2880218, y:6.8769461},
            geo::coord! {x:52.2883900, y:6.8739277},
        );
        assert_eq!(circle_line_intersect(&circle_center, radius, &line), true);
    }
}
