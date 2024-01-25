// Open Set: This is where you store nodes that are yet to be evaluated.
//      A binary heap (BinaryHeap in Rust's standard library) is a good choice for the open set
//      as it allows you to efficiently get the node with the lowest f value, which is essential for A*.
// Closed Set: This set contains nodes that have already been evaluated.
//      A HashSet is typically used for the closed set to allow for quick lookups to check if
//      a node has already been processed.

//! A* path planner

use crate::mav_link_plan;

use dubins_paths::{DubinsPath, PosRot, Result as DubinsResult};
use geo;
use geographiclib_rs::{DirectGeodesic, Geodesic, InverseGeodesic};
use std::collections::BinaryHeap;
use std::collections::HashMap;
use std::f64::consts::PI;
use std::rc::Rc;

mod planning_waypoints;
use planning_waypoints::{GeospatialPose, Node};

/// Data for open set, closed set, goal pose, and optimal path for the A* planner.
pub struct AStarPlanner {
    open_set: BinaryHeap<Rc<Node>>,
    closed_set: HashMap<GeospatialPose, Rc<Node>>,
    goal_pose: GeospatialPose,
    optimal_path: Vec<Rc<Node>>,
    geofence_lines: Vec<geo::LineString>,
}

impl AStarPlanner {
    pub fn new(
        start_point: geo::Point,
        start_heading: f64,
        end_point: geo::Point,
    ) -> Result<Self, String> {
        let start_pose = GeospatialPose {
            position: start_point,
            height: 120.0,
            heading: start_heading,
        };
        let goal_pose = GeospatialPose {
            position: end_point,
            height: 120.0,
            heading: 0.0,
        };

        if !start_pose.is_valid() {
            return Err(String::from("Ongeldige beginpositie."));
        }
        if !goal_pose.is_valid() {
            return Err(String::from("Ongeldige eindpositie."));
        }

        let mut a_star_planner = AStarPlanner {
            open_set: BinaryHeap::new(),
            closed_set: HashMap::new(),
            goal_pose: goal_pose,
            optimal_path: Vec::new(),
            geofence_lines: Vec::new(),
        };

        let cost_to_goal = calculate_h(&start_pose, &goal_pose);

        let start_node_rc = Rc::new(planning_waypoints::Node {
            pose: start_pose,
            traveled_distance: 0.0,
            steering: 0.0,
            steering_integral: 0.0,
            g: 0.0,
            h: cost_to_goal,
            f: cost_to_goal + 0.0,
            parent: None,
        });

        a_star_planner.open_set.push(start_node_rc);

        Ok(a_star_planner)
    }

    pub fn calculate_path(&mut self) {
        self.optimal_path.clear();
        let geod = Geodesic::wgs84();

        let distance_increment = 140.0;

        let mut path_not_found: bool = true;
        let mut point_in_reach_of_goal: BinaryHeap<Rc<Node>> = BinaryHeap::new();
        while path_not_found {
            // Pull top position from Open Set
            let parent = self.open_set.pop().unwrap();

            // calculate new positions
            let mut next_poses = parent.pose.calculate_next_poses(distance_increment);
            // Check validity of new positions
            // Check if new position are in Closed Set (already treated)
            next_poses.retain(|&pose| pose.is_valid() && !self.closed_set.contains_key(&pose));

            // Calculate node information of new position
            for pose in next_poses.iter() {
                let steering = pose.heading - parent.pose.heading;
                let steering_integral = parent.steering_integral + steering.abs();

                let cost_so_far = calculate_g(&pose, &parent, distance_increment);
                let cost_to_goal = calculate_h(&pose, &self.goal_pose);

                let node = Rc::new(Node {
                    pose: pose.clone(),
                    traveled_distance: parent.traveled_distance + distance_increment,
                    steering: steering,
                    steering_integral: steering_integral,
                    g: cost_so_far,
                    h: cost_to_goal,
                    f: cost_so_far + cost_to_goal,
                    parent: Some(Rc::clone(&parent)),
                });

                // Check if new positions is goal
                let (distance, _, _, _) = geod.inverse(
                    pose.position.x(),
                    pose.position.y(),
                    self.goal_pose.position.x(),
                    self.goal_pose.position.y(),
                );
                if distance < distance_increment {
                    path_not_found = false;
                    point_in_reach_of_goal.push(Rc::clone(&node));
                    self.closed_set.insert(node.pose, node);
                } else {
                    // Push new positions to Open Set
                    self.open_set.push(node);
                }
            }

            // Push top position to Closed Set
            self.closed_set.insert(parent.pose, parent);
        }
        self.optimal_path
            .push(Rc::clone(&point_in_reach_of_goal.pop().unwrap()));

        // Fill the vector with waypoints from goal pose to start pose.
        loop {
            let node = self
                .closed_set
                .get(&self.optimal_path[self.optimal_path.len() - 1].pose)
                .unwrap();
            match &node.parent {
                Some(value) => self.optimal_path.push(Rc::clone(&value)),
                None => break,
            }
        }
    }

    pub fn get_optimal_path(&self) -> Vec<geo::Point> {
        let mut optimal_path: Vec<geo::Point> = Vec::new();

        for point in self.optimal_path.iter().rev() {
            optimal_path.push(point.pose.position);
        }

        optimal_path.push(self.goal_pose.position);

        println!(
            "# waypoints: {} | # closed set {} | # open set {}",
            optimal_path.len(),
            self.closed_set.len(),
            self.open_set.len()
        );

        return optimal_path;
    }

    pub fn get_all_points(&self) -> Vec<geo::Point> {
        let mut all_points: Vec<geo::Point> = Vec::new();

        for node in self.closed_set.values() {
            all_points.push(node.pose.position);
        }

        // for node in self.open_set.iter() {
        //     all_points.push(node.pose.position);
        // }

        return all_points;
    }

    // pub fn fill_geofence(&self, geo_fence: &mav_link_plan::GeoFence) {
    //     for geo_fence_polygon in geo_fence.polygons.iter() {
    //         let p_last = geo_fence_polygon.polygon[geo_fence_polygon.polygon.len() - 1];
    //         for test in geo_fence_polygon.polygon.iter() {

    //             self.geofence_lines.push(lin)
    //         }
    //     }
    // }
}

impl GeospatialPose {
    fn calculate_next_poses(&self, distance_increment: f64) -> Vec<GeospatialPose> {
        let mut next_poses = Vec::new();

        // turn 90 degree left
        // next_poses.push(self.calculate_next_pose(-90.0, distance_increment));
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
        // next_poses.push(self.calculate_next_pose(90.0, distance_increment));

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
            // } else {
            // new_heading;
        }

        // println!(
        //     "heading {} | new heading {} | angle {}",
        //     self.heading, new_heading, angle
        // );

        return new_heading;
    }

    fn is_valid(&self) -> bool {
        true
    }
}

/// Calculate chord length
///
/// Fixed wing drone flies on circle arcs. Next position is calculated in a straight line.
/// *Limitation*: heading range is [-90...90] degrees.
///
/// * arc_length: Distance traveled over circle arc.
/// * angle_degrees: Heading change.
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
fn calculate_g(child_pose: &GeospatialPose, parent_node: &Node, distance_increment: f64) -> f64 {
    parent_node.traveled_distance + distance_increment
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

    let mut new_heading = (pose.heading - azimuth).abs();
    if new_heading > 180.0 {
        new_heading = 360.0 - new_heading;
    }
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

    distance
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
