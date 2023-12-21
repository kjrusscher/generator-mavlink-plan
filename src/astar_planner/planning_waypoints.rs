use std::cmp::Ordering;
// use std::collections::{BinaryHeap, HashSet};
use std::hash::{Hash, Hasher};
use std::rc::Rc;

use geo;

#[derive(Clone, Debug)]
pub struct GeospatialPose {
    position: geo::Point,
    heading: f64, // heading at this point
    height: f64,
}

#[derive(Clone, Debug)]
pub struct Node {
    pose: GeospatialPose,
    traveled_distance: f64,    // Make distance between point the same
    steering: f64,              // heading - heading parent
    steeringIntegral: f64,      // sum(|steering|)
    f: f64,                     // total cost g+h
    g: f64,                     // cost so far
    h: f64,                     // cost to goal
    parent: Option<Rc<Node>>,
}

impl GeospatialPose {
    pub fn is_valid() -> bool {
        true
    }
}

// =begin=========== Code for BinaryHeap and HashSet ==================begin=

fn round_point(value: f64) -> f64 {
    // 3 decimal places for circa 100 meter resolution
    (value * 1000.0).round() / 1000.0
}

fn round_height(value: f64) -> f64 {
    // 10 meter resolution
    (value * 10.0).round() / 10.0
}

fn round_heading(value: f64) -> f64 {
    // 10 degrees resolution
    (value * 10.0).round() / 10.0
}

impl PartialEq for Node {
    fn eq(&self, other: &Self) -> bool {
        self.f == other.f
    }
}

impl Eq for Node {}

impl Ord for Node {
    fn cmp(&self, other: &Self) -> Ordering {
        other.f.cmp(&self.f) // reverse order
    }
}

impl PartialOrd for Node {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl Hash for Node {
    fn hash<H: Hasher>(&self, state: &mut H) {
        geo::Point::new(round_point(self.pose.position.x), round_point(self.pose.position.y)).hash(state);
        round_height(self.pose.height).to_bits().hash(state);
        round_heading(self.pose.heading).to_bits().hash(state); // Convert to_bits for stable hashing of floats
    }
}

// =end============= Code for BinaryHeap and HashSet ====================end=
