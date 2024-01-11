// Open Set: This is where you store nodes that are yet to be evaluated.
//      A binary heap (BinaryHeap in Rust's standard library) is a good choice for the open set
//      as it allows you to efficiently get the node with the lowest f value, which is essential for A*.
// Closed Set: This set contains nodes that have already been evaluated.
//      A HashSet is typically used for the closed set to allow for quick lookups to check if
//      a node has already been processed.

/// A* Planner code
use geo;
use std::collections::BinaryHeap;
use std::collections::HashMap;
use std::rc::Rc;

mod planning_waypoints;
use planning_waypoints::{GeospatialPose, Node};

struct AStarPlanner {
    open_set: BinaryHeap<Node>,
    closed_set: HashMap<Node, Node>,
    goal_pose: GeospatialPose,
}

impl AStarPlanner {
    pub fn new(
        start_point: geo::Point,
        start_heading: f64,
        end_point: geo::Point,
    ) -> Result<Self, std::io::Error> {
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
            return Err("Ongeldige beginpositie.");
        }
        if !goal_pose.is_valid() {
            return Err("Ongeldige eindpositie.");
        }

        let a_star_planner = AStarPlanner {
            open_set: BinaryHeap::new(),
            closed_set: HashMap::new(),
            goal_pose: goal_pose,
        };

        let cost_to_goal = start_point.euclidian_distance(goal);

        let start_node = planning_waypoints::Node {
            position: start_point,
            heading: start_heading,
            traveled_distance: 0.0,
            steering: 0.0,
            steeringIntegral: 0.0,
            g: 0.0,
            h: cost_to_goal,
            f: cost_to_goal,
            parent: None,
        };

        a_star_planner.open_set.push(start_node);

        Ok(a_star_planner)
    }

    pub fn calculate_path() -> Vec<Node> {
        loop {
            // Pull top position from Open Set
            let parent = Self.open_set.pop();

            // calculate new positions
            let mut next_poses = calculate_next_poses(&parent);
            // Check validity of new positions
            // Check if new position are in Closed Set (already treated)
            next_poses.retain(|&pose| pose.is_valid() && !Self.closed_set.contains(pose));
            // Check if new positions is end
            for pose in next_poses.iter() {
                if pose.is_equal(Self.goal_pose) {
                    println!("End is reached");
                }
            }

            // Calculate node information of new position
            for pose in next_poses.iter() {
                let cost_so_far = calculate_g(&pose, &parent); // distance plus steering plus diff steering
                let cost_to_goal = calculate_h(&pose, Self.goal_pose);

                let node = Node {
                    pose: pose,
                    traveled_distance: parent.traveled_distance + 200, // Make distance between point the same
                    steering: pose.heading - parent.heading,           // heading - heading parent
                    steeringIntegral: (pose.heading - parent.heading).abs(), // sum(|steering|)
                    g: cost_so_far,                                    // cost so far
                    h: cost_to_goal,                                   // cost to goal
                    f: cost_so_far + cost_to_goal,                     // total cost g+h
                    parent: Some(Rc::clone(&parent)),
                };
                // Push new positions to Open Set
                Self.open_set.push(node);
            }

            // Push top position to Closed Set
            Self.closed_set.insert(parent);
        }
    }
}

fn calculate_next_poses(pose: &GeospatialPose) -> Vec<GeospatialPose> {
    let mut next_poses = Vec::new();
    let g = Geodesic::wgs84();
    // Convert from wind direction system to mathematical system.
    let adjusted_degrees = pose.heading - 90.0;
    let heading_radian = adjusted_degrees * (std::f64::consts::PI / 180.0);
    
    // 250 meters straight ahead
    let (lat,lon,_) = g.direct(pose.position.x, pose.position.y, pose.heading, 250);
    let point_straight_ahead = geo::Point::new(lat, lon);
    let pose_straight_ahead = GeospatialPose {
        position: point_straight_ahead,
        heading: pose.heading,
        height: 120,
    };
    next_poses.push(pose_straight_ahead);

    // sharp left turn
    let x_increment = 145;
    let y_increment = 145;
    let point_sharp_left = geo::Point::new(
        pose.position.x + x_increment * heading_radian.cos() - y_increment * heading_radian.sin(),
        pose.position.y + x_increment * heading_radian.sin() + y_increment * heading_radian.cos(),
    );
    let heading_sharp_left = if pose.heading > 90 {
        heading - 90.0
    } else {
        heading + 270.0
    };
    let pose_sharp_left = GeospatialPose {
        position: point_sharp_left,
        heading: heading_sharp_left,
        height: 120,
    };
    next_poses.push(pose_sharp_left);

    // sharp right turn
    let x_increment = 145;
    let y_increment = 145;
    let point_sharp_right = geo::Point::new(
        pose.position.x + x_increment * heading_radian.cos() - y_increment * heading_radian.sin(),
        pose.position.y + x_increment * heading_radian.sin() + y_increment * heading_radian.cos(),
    );
    let heading_sharp_right = if pose.heading < 270 {
        heading + 90.0
    } else {
        heading - 270.0
    };
    let pose_sharp_right = GeospatialPose {
        position: point_sharp_right,
        heading: heading_sharp_right,
        height: 120,
    };
    next_poses.push(pose_sharp_right);

    return next_poses;
}

/// Calculate cost so far
fn calculate_g(child_pose: &GeospatialPose, parent_pose: &GeospatialPose) -> f64 {
    1.0
}

/// Calculate cost to goal
fn calculate_h(pose: &GeospatialPose, goal_pose: &GeospatialPose) -> f64 {
    1.0
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_calculate_next_poses() {

        let initial_pose = GeospatialPose{
            position: geo::Point::new(60.0,5.0),
            heading: 90.0,
            height: 120.0
        };

        // Exercise
        let next_poses = calculate_next_poses(&initial_pose);

        assert!

    }
}