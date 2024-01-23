// Open Set: This is where you store nodes that are yet to be evaluated.
//      A binary heap (BinaryHeap in Rust's standard library) is a good choice for the open set
//      as it allows you to efficiently get the node with the lowest f value, which is essential for A*.
// Closed Set: This set contains nodes that have already been evaluated.
//      A HashSet is typically used for the closed set to allow for quick lookups to check if
//      a node has already been processed.

//! A* path planner
use dubins_paths::{DubinsPath, PosRot, Result as DubinsResult};
use geo;
use geographiclib_rs::{DirectGeodesic, Geodesic, InverseGeodesic};
use std::collections::BinaryHeap;
use std::collections::HashMap;
use std::f64::consts::PI as PI;
use std::rc::Rc;

mod planning_waypoints;
use planning_waypoints::{GeospatialPose, Node};

/// Data for open set, closed set, goal pose, and optimal path for the A* planner.
pub struct AStarPlanner {
    open_set: BinaryHeap<Rc<Node>>,
    closed_set: HashMap<GeospatialPose, Rc<Node>>,
    goal_pose: GeospatialPose,
    optimal_path: Vec<Rc<Node>>,
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
        };

        let cost_to_goal = calculate_h(&start_pose, &goal_pose);

        let start_node_rc = Rc::new(planning_waypoints::Node {
            pose: start_pose,
            traveled_distance: 0.0,
            steering: 0.0,
            steering_integral: 0.0,
            g: 0.0,
            h: cost_to_goal,
            f: cost_to_goal,
            parent: None,
        });

        a_star_planner.open_set.push(start_node_rc);

        Ok(a_star_planner)
    }

    pub fn calculate_path(&mut self) {
        self.optimal_path.clear();

        let distance_increment = 300.0;

        let mut path_not_found: bool = true;
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

                let cost_so_far = calculate_g(&pose, &parent, distance_increment); // distance plus steering plus diff steering
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
                if pose == &self.goal_pose {
                    path_not_found = false;
                    println!("Goal pose is reached");
                    self.optimal_path.push(Rc::clone(&node));
                    self.closed_set.insert(node.pose, node);
                    break;
                } else {
                    // Push new positions to Open Set
                    self.open_set.push(node);
                }
            }

            // Push top position to Closed Set
            self.closed_set.insert(parent.pose, parent);
        }

        // Fill the vector with waypoints from goal pose to start pose.
        let mut pose = self.optimal_path[0].pose;
        loop {
            let node = self.closed_set.get(&pose).unwrap();
            self.optimal_path.push(Rc::clone(node));

            if let Some(value) = &node.parent {
                pose = value.pose;
            } else {
                break;
            }
        }
    }
}

impl GeospatialPose {
    fn calculate_next_poses(&self, distance_travelled: f64) -> Vec<GeospatialPose> {
        let mut next_poses = Vec::new();
        let g = Geodesic::wgs84();

        let heading_left_90 = self.calculate_new_heading(-90.0);
        let heading_left_45 = self.calculate_new_heading(-45.0);
        let heading_right_45 = self.calculate_new_heading(45.0);
        let heading_right_90 = self.calculate_new_heading(90.0);
        let _chord_length_45 = chord_length(distance_travelled, 45.0);
        let chord_length_90 = chord_length(distance_travelled, 90.0);

        // 250 meters straight ahead
        let (mut lat, mut lon, _) = g.direct(
            self.position.x(),
            self.position.y(),
            self.heading,
            distance_travelled,
        );
        let point_straight_ahead = geo::Point::new(lat, lon);
        let pose_straight_ahead = GeospatialPose {
            position: point_straight_ahead,
            heading: self.heading,
            height: 120.0,
        };
        next_poses.push(pose_straight_ahead);

        // turn 90 degree left
        (lat, lon, _) = g.direct(
            self.position.x(),
            self.position.y(),
            heading_left_45,
            chord_length_90,
        );
        let point_left_90 = geo::Point::new(lat, lon);
        let pose_left_90 = GeospatialPose {
            position: point_left_90,
            heading: heading_left_90,
            height: 120.0,
        };
        next_poses.push(pose_left_90);

        // turn 90 degrees right
        (lat, lon, _) = g.direct(
            self.position.x(),
            self.position.y(),
            heading_right_45,
            chord_length_90,
        );
        let point_right_90 = geo::Point::new(lat, lon);
        let pose_right_90 = GeospatialPose {
            position: point_right_90,
            heading: heading_right_90,
            height: 120.0,
        };
        next_poses.push(pose_right_90);

        return next_poses;
    }

    /// Calculates new heading and put it in 0-360 range.
    fn calculate_new_heading(&self, angle: f64) -> f64 {
        if angle < -360.0 || angle > 360.0 {
            panic!("Angle is outside 0.0-360.0 range. Indicates error in code")
        }
        let new_heading = self.heading + angle;
        if new_heading < 0.0 {
            new_heading + 360.0
        } else if new_heading > 360.0 {
            new_heading - 360.0
        } else {
            new_heading
        }
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
/// * arc_length - Distance traveled over circle arc.
/// * angle_degrees - Heading change.
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
    let g = Geodesic::wgs84();
    let (lat1, lon1) = (pose.position.x(), pose.position.y()); // Start position
    let (lat2, lon2) = (goal_pose.position.x(), goal_pose.position.y()); // Goal Position
    let (_, d_m, az1, _) = g.inverse(lat1, lon1, lat2, lon2);

    let mut new_heading = pose.heading - az1;
    if new_heading < 0.0 {
        new_heading += 360.0;
    } else if new_heading > 360.0 {
        new_heading -= 360.0;
    }
    let new_heading_rad = new_heading * PI / 360.0; 

    let start_position: PosRot = PosRot::from_f32(0.0, 0.0, new_heading_rad as f32);
    let goal_position: PosRot = PosRot::from_f32(d_m as f32, 0.0, 0.0);
    let shortest_path_possible = DubinsPath::shortest_from(
        start_position,
        goal_position,
        140.0,
    )
    .unwrap();

    shortest_path_possible.length() as f64
}

// #[cfg(test)]
// mod tests {
//     use super::*;

//     #[test]
//     fn test_calculate_next_poses() {

//         let initial_pose = GeospatialPose{
//             position: geo::Point::new(60.0,5.0),
//             heading: 90.0,
//             height: 120.0
//         };

//         // Exercise
//         let next_poses = calculate_next_poses(&initial_pose);

//         assert!

//     }
// }
