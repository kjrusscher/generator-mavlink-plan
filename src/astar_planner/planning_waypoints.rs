use std::cmp::Ordering;
// use std::collections::{BinaryHeap, HashSet};
use std::hash::{Hash, Hasher};
use std::rc::Rc;

use geo;

#[derive(Clone, Debug, Copy)]
pub struct GeospatialPose {
    pub position: geo::Point,
    pub heading: f64,
    pub height: f64,
}

#[derive(Clone, Debug)]
pub struct Node {
    pub pose: GeospatialPose,
    pub traveled_distance: f64,    // Make distance between point the same
    pub steering: f64,              // heading - heading parent
    pub steering_integral: f64,      // sum(|steering|)
    pub f: f64,                     // total cost g+h
    pub g: f64,                     // cost so far
    pub h: f64,                     // cost to goal
    pub parent: Option<Rc<Node>>,
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

impl Eq for Node {}

impl PartialEq for Node {
    fn eq(&self, other: &Self) -> bool {
        self.f == other.f
    }
}

impl Ord for Node {
    fn cmp(&self, other: &Self) -> Ordering {
        self.f.partial_cmp(&other.f).unwrap_or(Ordering::Equal)
    }
}

impl PartialOrd for Node {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl Eq for GeospatialPose {}

impl PartialEq for GeospatialPose {
    fn eq(&self, other: &Self) -> bool {
        round_point(self.position.x()) == round_point(other.position.x()) &&
        round_point(self.position.y()) == round_point(other.position.y()) &&
        round_height(self.height) == round_height(other.height) &&
        round_heading(self.heading) == round_heading(other.heading)
    }
}

impl Hash for GeospatialPose {
    fn hash<H: Hasher>(&self, state: &mut H) {
        geo::Point::new(round_point(self.position.x()).to_bits(), round_point(self.position.y()).to_bits()).hash(state);
        round_height(self.height).to_bits().hash(state);
        round_heading(self.heading).to_bits().hash(state); // Convert to_bits for stable hashing of floats
    }
}

// =end============= Code for BinaryHeap and HashSet ====================end=
