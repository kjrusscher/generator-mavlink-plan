use crate::mav_link_plan;
use geo;
use geo_types::coord;
use std::rc::Rc;

use super::planning_waypoints::{GeospatialPose, Node};
use super::AStarPlanner;

pub struct AStarPlannerBuilder {
    start_pose: Option<GeospatialPose>, // Waypoint where take off sequence ends
    goal_pose: Option<GeospatialPose>,  // Where do you want to fly
    end_pose: Option<GeospatialPose>,   // Waypoint where landing sequence starts
    optimal_path_from_start_to_goal: Vec<Rc<Node>>,
    optimal_path_from_goal_to_end: Vec<Rc<Node>>,
    geo_fences_polygon: Option<geo::MultiLineString>,
    geo_fences_circles: Option<Vec<geo::Point>>,
}

impl AStarPlannerBuilder {
    pub fn new() -> Self {
        AStarPlannerBuilder {
            start_pose: None,
            goal_pose: None,
            end_pose: None,
            optimal_path_from_start_to_goal: Vec::new(),
            optimal_path_from_goal_to_end: Vec::new(),
            geo_fences_circles: None,
            geo_fences_polygon: None,
        }
    }

    pub fn start(mut self, point: geo::Point, heading: f64) -> Self {
        self.start_pose = Some(GeospatialPose {
            position: point,
            heading: heading,
            height: 115.0,
        });
        self
    }

    pub fn end(mut self, point: geo::Point, heading: f64) -> Self {
        self.end_pose = Some(GeospatialPose {
            position: point,
            heading: heading,
            height: 115.0,
        });
        self
    }

    pub fn goal(mut self, point: &geo::Point) -> Self {
        self.goal_pose = Some(GeospatialPose {
            position: *point,
            heading: 0.0,
            height: 115.0,
        });
        self
    }

    /// Read a MavLink GeoFence structure and translate it to geo::Points and geo::LineStrings
    /// for use by the A* Planner.
    pub fn set_geo_fences(&mut self, geo_fences: &mav_link_plan::GeoFence) {
        let mut geo_fence_polygons_vector = vec![];
        for geo_fence_polygon in geo_fences.polygons.iter() {
            let mut polygon_vector = vec![];
            for point in geo_fence_polygon.polygon.iter() {
                polygon_vector.push(coord! {x: point[0], y: point[1]});
            }
            polygon_vector.push(polygon_vector[0]);
            geo_fence_polygons_vector.push(geo::LineString::new(polygon_vector));
        }
        self.geo_fences_polygon = Some(geo::MultiLineString::new(geo_fence_polygons_vector));

        self.geo_fences_circles = None;
        if geo_fences.circles.is_empty() {
            self.geo_fences_circles.get_or_insert_with(Vec::new);
        } else {
            for geo_fence_circle in geo_fences.circles.iter() {
                self.geo_fences_circles
                    .get_or_insert_with(Vec::new)
                    .push(geo::Point::new(
                        geo_fence_circle.circle.center[0],
                        geo_fence_circle.circle.center[1],
                    ));
            }
        }
    }

    pub fn build(self) -> Result<AStarPlanner, &'static str> {
        if let (
            Some(start_pose),
            Some(goal_pose),
            Some(end_pose),
            Some(geo_fences_circles),
            Some(geo_fences_polygon),
        ) = (
            self.start_pose,
            self.goal_pose,
            self.end_pose,
            self.geo_fences_circles.clone(),
            self.geo_fences_polygon.clone(),
        ) {
            Ok(AStarPlanner {
                start_pose,
                goal_pose,
                end_pose,
                optimal_path_from_start_to_goal: self.optimal_path_from_start_to_goal.clone(),
                optimal_path_from_goal_to_end: self.optimal_path_from_goal_to_end.clone(),
                geo_fences_circles,
                geo_fences_polygon,
            })
        } else {
            Err("Missing required fields for AStarPlanner")
        }
    }
}
