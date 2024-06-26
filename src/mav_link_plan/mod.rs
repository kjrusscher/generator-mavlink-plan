//! Data structs for the .plan file
use geo;
use geographiclib_rs::{DirectGeodesic, Geodesic};
use serde::{Deserialize, Serialize};
use serde_repr::{Deserialize_repr, Serialize_repr};

/// Enum mavlink::common::MavCmd does not serialize well to a number. This one does serialize correctly.
#[derive(Serialize_repr, Deserialize_repr)]
#[repr(u16)]
#[allow(non_camel_case_types)]
pub enum MavCmd {
    MAV_CMD_NAV_WAYPOINT = 16,
    MAV_CMD_NAV_LOITER_UNLIM = 17,
    MAV_CMD_NAV_LAND = 21,
    MAV_CMD_NAV_TAKEOFF = 22,
    MAV_CMD_DO_JUMP = 177,
    MAV_CMD_DO_VTOL_TRANSITION = 3000,
}

/// Enum mavlink::common::MavFrame does not serialize well to a number. This one does serialize correctly.
#[derive(Serialize_repr, Deserialize_repr)]
#[repr(u16)]
#[allow(non_camel_case_types)]
pub enum MavFrame {
    MAV_FRAME_GLOBAL = 0,
    MAV_FRAME_LOCAL_NED = 1,
    MAV_FRAME_MISSION = 2,
    MAV_FRAME_GLOBAL_RELATIVE_ALT = 3,
    MAV_FRAME_LOCAL_ENU = 4,
    MAV_FRAME_GLOBAL_INT = 5,
    MAV_FRAME_GLOBAL_RELATIVE_ALT_INT = 6,
    MAV_FRAME_LOCAL_OFFSET_NED = 7,
    MAV_FRAME_GLOBAL_TERRAIN_ALT = 10,
    MAV_FRAME_GLOBAL_TERRAIN_ALT_INT = 11,
    MAV_FRAME_BODY_FRD = 12,
    MAV_FRAME_LOCAL_FRD = 20,
    MAV_FRAME_LOCAL_FLU = 21,
}

/// Enum mavlink::common::MavFrame does not serialize well to a number. This one does serialize correctly.
#[derive(Serialize_repr, Deserialize_repr)]
#[repr(u16)]
#[allow(non_camel_case_types)]
pub enum MavAltitudeMode {
    MAV_ALTITUDE_MODE_MIXED = 0,
    MAV_ALTITUDE_MODE_RELATIVE_TO_LAUNCH = 1,
    MAV_ALTITUDE_MODE_ASML = 2,
    MAV_ALTITUDE_MODE_CALC_ABOVE_TERRAIN = 3,
}

/// Used in GeoFenceCirle
#[derive(Serialize, Deserialize, Clone)]
pub struct Circle {
    pub center: [f64; 2],
    pub radius: f64,
}

/// Used in GeoFence
#[derive(Serialize, Deserialize, Clone)]
pub struct GeoFenceCircle {
    pub circle: Circle,
    pub inclusion: bool,
    pub version: i32,
}

/// Used in GeoFence
#[derive(Serialize, Deserialize, Clone)]
pub struct GeoFencePolygon {
    pub inclusion: bool,
    pub polygon: Vec<[f64; 2]>,
    pub version: i32,
}

/// Used in MavLinkPlan
#[derive(Serialize, Deserialize, Clone)]
pub struct GeoFence {
    pub circles: Vec<GeoFenceCircle>,
    pub polygons: Vec<GeoFencePolygon>,
    pub version: i32,
}

/// Used in MavLinkPlan
#[derive(Serialize, Deserialize)]
pub struct RallyPoints {
    pub points: Vec<[f64; 3]>,
    pub version: i32,
}

/// Used in Mission
#[derive(Serialize, Deserialize)]
#[allow(non_snake_case)]
pub struct MavLinkSimpleItem {
    pub AMSLAltAboveTerrain: Option<i32>,
    pub Altitude: Option<i32>,
    pub AltitudeMode: Option<MavAltitudeMode>,
    pub MISSION_ITEM_ID: Option<String>,
    pub autoContinue: bool,
    pub command: MavCmd,
    pub doJumpId: Option<i32>,
    pub frame: MavFrame,
    pub params: [Option<f64>; 7],
    #[serde(rename = "type")]
    pub type_name: String,
}

/// Used in MavLinkPlan
#[derive(Serialize, Deserialize)]
#[allow(non_snake_case)]
pub struct Mission {
    pub cruiseSpeed: i32,
    pub firmwareType: i32,
    pub globalPlanAltitudeMode: MavAltitudeMode,
    pub hoverSpeed: i32,
    pub items: Vec<MavLinkSimpleItem>,
    pub plannedHomePosition: [f64; 3],
}

/// Top level item from which .plan is generated
#[derive(Serialize, Deserialize)]
#[allow(non_snake_case)]
pub struct MavLinkPlan {
    pub fileType: String,
    pub geoFence: GeoFence,
    pub groundStation: String,
    pub mission: Mission,
    pub rallyPoints: RallyPoints,
    pub version: u8,
}

impl Default for MavLinkSimpleItem {
    fn default() -> Self {
        let item = MavLinkSimpleItem {
            AMSLAltAboveTerrain: Some(115),
            Altitude: Some(115),
            AltitudeMode: Some(MavAltitudeMode::MAV_ALTITUDE_MODE_RELATIVE_TO_LAUNCH),
            MISSION_ITEM_ID: None,
            autoContinue: true,
            command: MavCmd::MAV_CMD_NAV_WAYPOINT,
            doJumpId: None,
            frame: MavFrame::MAV_FRAME_GLOBAL_RELATIVE_ALT,
            params: [
                Some(0.0),
                Some(0.0),
                Some(0.0),
                None,
                None,
                None,
                Some(115.0),
            ],
            type_name: String::from("SimpleItem"),
        };

        item
    }
}

impl Default for MavLinkPlan {
    fn default() -> Self {
        let geo_fence = GeoFence {
            circles: vec![],
            polygons: vec![],
            version: 2,
        };

        let rally_points = RallyPoints {
            points: vec![[52.282504959546564, 6.898488645574076, 0.0]],
            version: 2,
        };

        let mission = Mission {
            cruiseSpeed: 28,
            firmwareType: 12,
            globalPlanAltitudeMode: MavAltitudeMode::MAV_ALTITUDE_MODE_RELATIVE_TO_LAUNCH,
            hoverSpeed: 8,
            items: vec![],
            plannedHomePosition: [52.2825397, 6.8984103, 40.44],
        };

        MavLinkPlan {
            fileType: String::from("Plan"),
            version: 1,
            geoFence: geo_fence,
            groundStation: String::from("QGroundControl"),
            mission: mission,
            rallyPoints: rally_points,
        }
    }
}

impl MavLinkPlan {
    pub fn new(planned_home: [f64; 3]) -> Self {
        let mut plan = MavLinkPlan::default();

        // plan.mission.plannedHomePosition = [52.2825397, 6.8984103, 40.44];
        plan.mission.plannedHomePosition = planned_home;

        plan
    }

    pub fn add_take_off_sequence(mut self, wind_direction: f64, home_position: geo::Point) -> Self {
        self.add_special_waypoint(
            Some(30),
            MavCmd::MAV_CMD_NAV_TAKEOFF,
            [
                Some(0.0),
                Some(0.0),
                Some(0.0),
                None,
                Some(home_position.x()),
                Some(home_position.y()),
                Some(17.0),
            ],
        );

        let vtol_transition_position = calculate_vtol_transistion_position(home_position);
        self.add_waypoint(
            60,
            vtol_transition_position.x(),
            vtol_transition_position.y(),
        );

        self.add_special_waypoint(
            None,
            MavCmd::MAV_CMD_DO_VTOL_TRANSITION,
            [
                Some(4.0),
                Some(0.0),
                Some(0.0),
                Some(0.0),
                Some(0.0),
                Some(0.0),
                Some(0.0),
            ],
        );

        let (last_waypoint, _) = get_take_off_waypoint(wind_direction, home_position);
        self.add_waypoint(60, last_waypoint.x(), last_waypoint.y());

        self
    }

    pub fn add_landing_sequence(mut self, wind_direction: f64, home_position: geo::Point) -> Self {
        let (first_waypoint, _) = get_landing_waypoint(wind_direction, home_position);
        self.add_waypoint(60, first_waypoint.x(), first_waypoint.y());

        self.add_special_waypoint(
            None,
            MavCmd::MAV_CMD_DO_VTOL_TRANSITION,
            [
                Some(3.0),
                Some(0.0),
                Some(0.0),
                Some(0.0),
                Some(0.0),
                Some(0.0),
                Some(0.0),
            ],
        );

        let vtol_transition_position = calculate_vtol_transistion_position(home_position);
        self.add_waypoint(
            60,
            vtol_transition_position.x(),
            vtol_transition_position.y(),
        );

        self.add_special_waypoint(
            Some(0),
            MavCmd::MAV_CMD_NAV_LAND,
            [
                Some(0.0),
                Some(1.0),
                Some(0.0),
                None,
                Some(home_position.x()),
                Some(home_position.y()),
                Some(0.0),
            ],
        );

        self
    }

    pub fn add_path(mut self, path: &[geo::Point]) -> Self {
        for point in path.iter() {
            self.add_waypoint(115, point.x(), point.y());
        }
        self
    }

    /// Add goal position as loiter point.
    pub fn add_goal_position(mut self, goal: &geo::Point) -> Self {
        self.add_special_waypoint(
            Some(115),
            MavCmd::MAV_CMD_NAV_LOITER_UNLIM,
            [
                Some(0.0),
                Some(0.0),
                Some(140.0),
                None,
                Some(goal.x()),
                Some(goal.y()),
                Some(115.0),
            ],
        );
        self
    }

    fn add_waypoint(&mut self, height: i32, latitude: f64, longitude: f64) {
        let mut waypoint = MavLinkSimpleItem::default();
        let id: i32 = (self.mission.items.len() as i32) + 1;
        waypoint.AMSLAltAboveTerrain = Some(height);
        waypoint.Altitude = Some(height);
        waypoint.MISSION_ITEM_ID = Some(id.to_string());
        waypoint.doJumpId = Some(id);
        waypoint.params = [
            Some(0.0),
            Some(0.0),
            Some(0.0),
            None,
            Some(latitude),
            Some(longitude),
            Some(f64::from(height)),
        ];

        self.mission.items.push(waypoint);
    }

    fn add_special_waypoint(
        &mut self,
        height: Option<i32>,
        command: MavCmd,
        params: [Option<f64>; 7],
    ) {
        let mut waypoint = MavLinkSimpleItem::default();
        let id: i32 = (self.mission.items.len() as i32) + 1;
        waypoint.AMSLAltAboveTerrain = height;
        waypoint.Altitude = height;
        waypoint.MISSION_ITEM_ID = Some(id.to_string());
        waypoint.command = command;
        waypoint.doJumpId = Some(id);
        waypoint.params = params;

        self.mission.items.push(waypoint);
    }

    pub fn copy_geo_fences(&mut self, source: &Self) {
        self.geoFence = source.geoFence.clone();
    }
}

/// Get last waypoint of take off sequence
pub fn get_take_off_waypoint(
    wind_direction: f64,
    home_position: geo::Point,
) -> (geo::Point<f64>, f64) {
    let (direction_take_off_point, _) = adjust_wind_direction(wind_direction);
    let (take_off_direction, _) = adjust_wind_direction(wind_direction);

    let vtol_transition_position = calculate_vtol_transistion_position(home_position);
    let geod = Geodesic::wgs84();
    let (lat, lon) = geod.direct(
        vtol_transition_position.x(),
        vtol_transition_position.y(),
        direction_take_off_point,
        210.0,
    );
    (geo::Point::new(lat, lon), take_off_direction)
}

/// Get first waypoint of landings sequence
pub fn get_landing_waypoint(
    wind_direction: f64,
    home_position: geo::Point,
) -> (geo::Point<f64>, f64) {
    let (direction_landing_point, is_direction_adjusted) =
        adjust_wind_direction(calculate_opposite_wind_direction(wind_direction));
    let landing_direction = if is_direction_adjusted {
        calculate_opposite_wind_direction(wind_direction)
    } else {
        wind_direction
    };

    let vtol_transition_position = calculate_vtol_transistion_position(home_position);
    let geod = Geodesic::wgs84();
    let (lat, lon) = geod.direct(
        vtol_transition_position.x(),
        vtol_transition_position.y(),
        direction_landing_point,
        210.0,
    );
    (geo::Point::new(lat, lon), landing_direction)
}

fn adjust_wind_direction(wind_direction: f64) -> (f64, bool) {
    if wind_direction > 90.0 && wind_direction < 200.0 {
        (calculate_opposite_wind_direction(wind_direction), true)
    } else {
        (wind_direction, false)
    }
}

/// Calculates the opposite wind direction.
///
/// Given a wind direction in degrees, this function computes its opposite.
/// The wind direction is expected to be a value between 0 and 360 degrees,
/// where 0 or 360 represents North, 90 represents East, 180 represents South,
/// and 270 represents West. The opposite wind direction is calculated by
/// adding or subtracting 180 degrees, ensuring the result stays within the
/// 0 to 360 degrees range.
///
/// # Arguments
///
/// * `wind_direction` - A floating-point number representing the current wind direction in degrees.
///
/// # Returns
///
/// A floating-point number representing the opposite wind direction in degrees.
///
/// # Examples
///
/// ```
/// let current_wind_direction = 190.0;
/// let opposite_wind_direction = calculate_opposite_wind_direction(current_wind_direction);
/// assert_eq!(opposite_wind_direction, 10.0);
/// ```
fn calculate_opposite_wind_direction(wind_direction: f64) -> f64 {
    if wind_direction > 180.0 {
        wind_direction - 180.0
    } else {
        wind_direction + 180.0
    }
}

fn calculate_vtol_transistion_position(home_position: geo::Point) -> geo::Point {
    let geod = Geodesic::wgs84();
    let (lat, lon) = geod.direct(home_position.x(), home_position.y(), 60.0, 50.0);
    geo::Point::new(lat, lon)
}
