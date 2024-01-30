//! Data structs for the .plan file
use geo;
// use geographiclib_rs::{DirectGeodesic, Geodesic};
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

/// Used in GeoFenceCirle
#[derive(Serialize, Deserialize)]
pub struct Circle {
    pub center: [f64; 2],
    pub radius: f64,
}

/// Used in GeoFence
#[derive(Serialize, Deserialize)]
pub struct GeoFenceCircle {
    pub circle: Circle,
    pub inclusion: bool,
    pub version: i32,
}

/// Used in GeoFence
#[derive(Serialize, Deserialize)]
pub struct GeoFencePolygon {
    pub inclusion: bool,
    pub polygon: Vec<[f64; 2]>,
    pub version: i32,
}

/// Used in MavLinkPlan
#[derive(Serialize, Deserialize)]
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
    pub AltitudeMode: i32,
    pub MISSION_ITEM_ID: Option<String>,
    pub autoContinue: bool,
    pub command: MavCmd,
    pub doJumpId: Option<i32>,
    pub frame: i32,
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
    pub globalPlanAltitudeMode: i32,
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
            AMSLAltAboveTerrain: Some(120),
            Altitude: Some(120),
            AltitudeMode: 1,
            MISSION_ITEM_ID: None,
            autoContinue: true,
            command: MavCmd::MAV_CMD_NAV_WAYPOINT,
            doJumpId: None,
            frame: 3,
            params: [
                Some(0.0),
                Some(0.0),
                Some(0.0),
                None,
                Some(52.2825397),
                Some(6.8984103),
                Some(120.0),
            ],
            type_name: String::from("SimpleItem"),
        };

        item
    }
}

impl Default for MavLinkPlan {
    fn default() -> Self {
        let geo_fence_polygon_grens = GeoFencePolygon {
            inclusion: true,
            version: 1,
            polygon: vec![
                [52.28295542244744, 6.8565871319299845],
                [52.285431953652584, 6.86156240560706],
                [52.2896098479409, 6.8688319693996505],
                [52.29227492433401, 6.875640979066702],
                [52.2937453230195, 6.883632543793112],
                [52.29268600915777, 6.888311871426254],
                [52.294277072434966, 6.889956775263698],
                [52.293939049265504, 6.896312607978928],
                [52.29294882408129, 6.902284711120359],
                [52.28900135729954, 6.917255476279564],
                [52.2714465433939, 6.876881353338064],
                [52.27574362849022, 6.869559476902992],
                [52.27691573983945, 6.868882808809673],
                [52.27793930785583, 6.867776234580788],
                [52.27856267193794, 6.865940640293019],
                [52.27956352880396, 6.862960555652251],
                [52.281442698183156, 6.8598604985957365],
            ],
        };
        let geo_fence_polygon_spoorlijn = GeoFencePolygon {
            inclusion: false,
            version: 1,
            polygon: vec![
                [52.28397132247375, 6.863886719371692],
                [52.2911115585158, 6.8829664192158475],
                [52.29095744346746, 6.883631641810979],
                [52.28354519237769, 6.863920220780301],
            ],
        };

        let geo_fence = GeoFence {
            circles: vec![],
            polygons: vec![geo_fence_polygon_grens, geo_fence_polygon_spoorlijn],
            version: 2,
        };

        let rally_points = RallyPoints {
            points: vec![[52.282504959546564, 6.898488645574076, 0.0]],
            version: 2,
        };

        let mission = Mission {
            cruiseSpeed: 28,
            firmwareType: 12,
            globalPlanAltitudeMode: 1,
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
    pub fn new(wind_direction_10m: i32, path: &Vec<geo::Point>) -> Self {
        let mut plan = MavLinkPlan::default();

        let direction_take_off = f64::from(wind_direction_10m);

        // let position_pilot = geo::Point::new(52.282418448042776, 6.898363269759585);
        let home_position_drone = geo::Point::new(52.282538406253, 6.898364382855505);

        plan.add_take_off_sequence(direction_take_off, home_position_drone);

        let number_of_waypoints = path.len();
        for (index, point) in path.iter().enumerate() {
            if index < (number_of_waypoints - 1) {
                plan.add_waypoint(120, point.x(), point.y());
            } else {
                // Add loiter point.
                plan.add_special_waypoint(
                    Some(120),
                    MavCmd::MAV_CMD_NAV_LOITER_UNLIM,
                    [
                        Some(0.0),
                        Some(0.0),
                        Some(140.0),
                        None,
                        Some(point.x()),
                        Some(point.y()),
                        Some(120.0),
                    ],
                );
            }
        }

        plan
    }

    fn add_take_off_sequence(&mut self, direction_take_off: f64, home_position_drone: geo::Point) {
        // let geod = Geodesic::wgs84();

        self.add_special_waypoint(
            Some(30),
            MavCmd::MAV_CMD_NAV_TAKEOFF,
            [
                Some(15.0),
                Some(0.0),
                Some(0.0),
                Some(direction_take_off),
                Some(home_position_drone.x()),
                Some(home_position_drone.y()),
                Some(30.0),
            ],
        );

        // let (mut lat, mut lon) =
        //     geod.direct(home_position_drone.x(), home_position_drone.y(), 30.0, 35.0);
        // let vtol_transition = geo::Point::new(lat, lon);
        // self.add_waypoint(60, vtol_transition.x(), vtol_transition.y());

        // self.add_special_waypoint(
        //     None,
        //     MavCmd::MAV_CMD_DO_VTOL_TRANSITION,
        //     [
        //         Some(4.0),
        //         Some(0.0),
        //         Some(0.0),
        //         Some(0.0),
        //         Some(0.0),
        //         Some(0.0),
        //         Some(0.0),
        //     ],
        // );

        // (lat, lon) = geod.direct(
        //     vtol_transition.x(),
        //     vtol_transition.y(),
        //     direction_take_off,
        //     210.0,
        // );
        // let current_point = geo::Point::new(lat, lon);
        // self.add_waypoint(60, current_point.x(), current_point.y());
    }

    // fn add_landing_sequence(&mut self, direction_take_off: f64, home_position_drone: geo::Point) {
    //     let geod = Geodesic::wgs84();

    //     let direction_landing: f64;
    //     if direction_take_off < 180.0 {
    //         direction_landing = direction_take_off + 180.0;
    //     } else {
    //         direction_landing = direction_take_off - 180.0;
    //     }

    //     let (mut lat, mut lon) =
    //         geod.direct(home_position_drone.x(), home_position_drone.y(), 30.0, 35.0);
    //     let vtol_transition = geo::Point::new(lat, lon);

    //     (lat, lon) = geod.direct(
    //         vtol_transition.x(),
    //         vtol_transition.y(),
    //         direction_landing,
    //         210.0,
    //     );
    //     let current_point = geo::Point::new(lat, lon);
    //     self.add_waypoint(60, current_point.x(), current_point.y());

    //     self.add_waypoint(60, vtol_transition.x(), vtol_transition.y());

    //     self.add_special_waypoint(
    //         None,
    //         MavCmd::MAV_CMD_DO_VTOL_TRANSITION,
    //         [
    //             Some(3.0),
    //             Some(0.0),
    //             Some(0.0),
    //             Some(0.0),
    //             Some(0.0),
    //             Some(0.0),
    //             Some(0.0),
    //         ],
    //     );

    //     self.add_special_waypoint(
    //         Some(0),
    //         MavCmd::MAV_CMD_NAV_LAND,
    //         [
    //             Some(0.0),
    //             Some(1.0),
    //             Some(0.0),
    //             None,
    //             Some(home_position_drone.x()),
    //             Some(home_position_drone.y()),
    //             Some(0.0),
    //         ],
    //     );
    // }

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
}
