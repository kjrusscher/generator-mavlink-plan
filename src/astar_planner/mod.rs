// Open Set: This is where you store nodes that are yet to be evaluated.
//      A binary heap (BinaryHeap in Rust's standard library) is a good choice for the open set
//      as it allows you to efficiently get the node with the lowest f value, which is essential for A*.
// Closed Set: This set contains nodes that have already been evaluated.
//      A HashSet is typically used for the closed set to allow for quick lookups to check if
//      a node has already been processed.

//! A* path planner

// use crate::mav_link_plan;

use dubins_paths::{DubinsPath, PosRot};
use geo;
use geo::algorithm::intersects::Intersects;
use geo::Contains;
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
    geo_fences_polygon: geo::MultiLineString,
    geo_fences_circles: Vec<geo::Point>,
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

        // let geo_fence_railway = geo::LineString::new(vec![
        //     geo::coord! {x:52.28397132247375, y: 6.863886719371692},
        //     geo::coord! {x:52.2911115585158, y: 6.8829664192158475},
        //     geo::coord! {x:52.29095744346746, y: 6.883631641810979},
        //     geo::coord! {x:52.28354519237769, y: 6.863920220780301},
        //     // repeat first point to close the loop
        //     geo::coord! {x:52.28397132247375, y: 6.863886719371692},
        // ]);
        // let geo_fence_railway_1 = geo::LineString::new(vec![
        //     geo::coord!{x:52.28397132247375, y: 6.863886719371692},
        //     geo::coord!{x:52.287510968119605, y: 6.8733916286129215},
        //     geo::coord!{x:52.28729986661546, y: 6.8737774312500335},
        //     geo::coord!{x:52.28354519237769, y: 6.863920220780301},
        //     geo::coord!{x:52.28397132247375, y: 6.863886719371692},
        // ]);
        // let geo_fence_railway_2 = geo::LineString::new(vec![
        //     geo::coord!{x:52.288720670228805, y: 6.876414304674057},
        //     geo::coord!{x:52.29175801033167, y: 6.884476228939008},
        //     geo::coord!{x:52.291441047529446, y: 6.885057644180449},
        //     geo::coord!{x:52.288457999488145, y: 6.876892999221496},
        //     geo::coord!{x:52.288720670228805, y: 6.876414304674057},
        // ]);

        let mut geo_fence_circle: Vec<geo::Point> = Vec::new();
        // 5 circles on railway
        // geo_fence_circle.push(geo::Point::new(52.28342150438976, 6.862740698960664));
        // geo_fence_circle.push(geo::Point::new(52.28517350285496, 6.867905173957723));
        // geo_fence_circle.push(geo::Point::new(52.28717792705092, 6.87317966396094));
        // geo_fence_circle.push(geo::Point::new(52.289274467917615, 6.878535125811709));
        // geo_fence_circle.push(geo::Point::new(52.29119602537627, 6.883766547104244));
        // 6 circles on railway
        geo_fence_circle.push(geo::Point::new(52.283105290274484, 6.861985238490377));
        geo_fence_circle.push(geo::Point::new(52.28466271451338, 6.866473775137109));
        geo_fence_circle.push(geo::Point::new(52.28644826062613, 6.871032565740023));
        geo_fence_circle.push(geo::Point::new(52.288131372606706, 6.875354239578542));
        geo_fence_circle.push(geo::Point::new(52.28968817161486, 6.8797506782361495));
        geo_fence_circle.push(geo::Point::new(52.29148311807836, 6.8843410345191955));

        let mut a_star_planner = AStarPlanner {
            open_set: BinaryHeap::new(),
            closed_set: HashMap::new(),
            goal_pose: goal_pose,
            optimal_path: Vec::new(),
            geo_fences_polygon: geo::MultiLineString::new(vec![
                geo_fence_border,
                // geo_fence_railway_1,
                // geo_fence_railway_2,
                // geo_fence_railway,
            ]),
            geo_fences_circles: geo_fence_circle,
            // geo_fences_circles: vec![],
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
            next_poses.retain(|&pose| {
                pose.is_valid(
                    &parent.pose,
                    &self.geo_fences_polygon,
                    &self.geo_fences_circles,
                ) && !self.closed_set.contains_key(&pose)
            });

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

        optimal_path
    }

    pub fn get_all_points(&self) -> Vec<geo::Point> {
        let mut all_points: Vec<geo::Point> = Vec::new();

        for node in self.closed_set.values() {
            all_points.push(node.pose.position);
        }

        // for node in self.open_set.iter() {
        //     all_points.push(node.pose.position);
        // }

        all_points
    }

    // pub fn fill_geofence(&self, geo_fence: &mav_link_plan::GeoFence) {
    //     for geo_fence_polygon in geo_fence.polygons.iter() {
    //         let p_last = geo_fence_polygon.polygon[geo_fence_polygon.polygon.len() - 1];
    //         for test in geo_fence_polygon.polygon.iter() {

    //             self.geo_fences_polygon.push(lin)
    //         }
    //     }
    // }
}

impl GeospatialPose {
    fn calculate_next_poses(&self, distance_increment: f64) -> Vec<GeospatialPose> {
        let mut next_poses = Vec::new();

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
                if circle_line_intersect(&point, 100.0, &drone_path) {
                    return false;
                }
            }
            true
        }
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
    // parent_node.g + distance_increment
    let steering_weight = 4.0;
    let steering = heading_difference(child_pose.heading, parent_node.pose.heading);
    parent_node.g + distance_increment + steering_weight * steering * steering
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

    // distance
    let steering_weight = 4.0;
    distance + steering_weight * new_heading * new_heading
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
/// * `circle_center` - The center of the circle as a geo::Point<f64>.
/// * `radius` - The radius of the circle as an f64.
/// * `line` - The line as a geo::Line<f64>.
///
/// # Returns
/// True if the line segment intersects the circle, false otherwise.
fn circle_line_intersect(circle_center: &geo::Point<f64>, radius: f64, line: &geo::Line<f64>) -> bool {
    let (x1, y1) = transform_geo_to_cartesian_coordinates(&circle_center, &line.start.into());
    let (x2, y2) = transform_geo_to_cartesian_coordinates(&circle_center, &line.end.into());
    
    let dx = x2 - x1;
    let dy = y2 - y1;

    // Coefficients for the quadratic equation Ax^2 + Bx + C = 0
    let a = dx.powi(2) + dy.powi(2);
    let b = 2.0 * (x1 * dx + y1 * dy);
    let c = x1.powi(2) + y1.powi(2) - radius.powi(2);

    // Discriminant
    let discriminant = b.powi(2) - 4.0 * a * c;

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

fn transform_geo_to_cartesian_coordinates(origin: &geo::Point<f64>, other_point: &geo::Point<f64>) -> (f64, f64) {
    let geod = Geodesic::wgs84();
    let (s12, az1, _, _) = geod.inverse(origin.x(), origin.y(), other_point.x(), other_point.y());

    (s12*az1.sin(), s12*az1.cos())
}