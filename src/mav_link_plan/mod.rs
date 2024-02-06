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
        // let geo_fence_polygon_spoorlijn = GeoFencePolygon {
        //     inclusion: false,
        //     version: 1,
        //     polygon: vec![
        //         [52.28397132247375, 6.863886719371692],
        //         [52.2911115585158, 6.8829664192158475],
        //         [52.29095744346746, 6.88363164version:1810979],
        //         [52.28354519237769, 6.863920220780301],
        //     ],
        // };
        // let geo_fence_polygon_spoorlijn_1 = GeoFencePolygon {
        //     inclusion: false,
        //     version: 1,
        //     polygon: vec![
        //         [52.28397132247375, 6.863886719371692],
        //         [52.287510968119605, 6.8733916286129215],
        //         [52.28729986661546, 6.8737774312500335],
        //         [52.28354519237769, 6.863920220780301],
        //     ],
        // };
        // let geo_fence_polygon_spoorlijn_2 = GeoFencePolygon {
        //     inclusion: false,
        //     version: 1,
        //     polygon: vec![
        //         [52.288720670228805, 6.876414304674057],
        //         [52.29175801033167, 6.884476228939008],
        //         [52.291441047529446, 6.885057644180449],
        //         [52.288457999488145, 6.876892999221496],
        //     ],
        // };
        let geo_fence_circle = vec![
            GeoFenceCircle {
                circle: Circle {
                    center: [52.283105290274484, 6.861985238490377],
                    radius: 140.0,
                },
                inclusion: false,
                version: 1,
            },
            GeoFenceCircle {
                circle: Circle {
                    center: [52.28466271451338, 6.866473775137109],
                    radius: 140.0,
                },
                inclusion: false,
                version: 1,
            },
            GeoFenceCircle {
                circle: Circle {
                    center: [52.28644826062613, 6.871032565740023],
                    radius: 140.0,
                },
                inclusion: false,
                version: 1,
            },
            GeoFenceCircle {
                circle: Circle {
                    center: [52.288131372606706, 6.875354239578542],
                    radius: 140.0,
                },
                inclusion: false,
                version: 1,
            },
            GeoFenceCircle {
                circle: Circle {
                    center: [52.28968817161486, 6.8797506782361495],
                    radius: 140.0,
                },
                inclusion: false,
                version: 1,
            },
            GeoFenceCircle {
                circle: Circle {
                    center: [52.29148311807836, 6.8843410345191955],
                    radius: 140.0,
                },
                inclusion: false,
                version: 1,
            },
        ];

        let geo_fence = GeoFence {
            circles: geo_fence_circle,
            polygons: vec![
                geo_fence_polygon_grens,
                // geo_fence_polygon_spoorlijn_1,
                // geo_fence_polygon_spoorlijn_2,
                // geo_fence_polygon_spoorlijn,
            ],
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
    pub fn new() -> Self {
        let mut plan = MavLinkPlan::default();

        plan.mission.plannedHomePosition = [52.2825397, 6.8984103, 40.44];

        plan
    }

    pub fn add_take_off_sequence(&mut self, wind_direction: f64) {
        self.add_special_waypoint(
            Some(30),
            MavCmd::MAV_CMD_NAV_TAKEOFF,
            [
                Some(0.0),
                Some(0.0),
                Some(0.0),
                None,
                Some(self.mission.plannedHomePosition[0]),
                Some(self.mission.plannedHomePosition[1]),
                Some(17.0),
            ],
        );

        self.add_waypoint(30, 52.282666320797524, 6.898523816647838);
        self.add_waypoint(60, 52.282812822205244, 6.899070950501141);

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

        let last_waypoint = get_take_off_waypoint(wind_direction);
        self.add_waypoint(60, last_waypoint.x(), last_waypoint.y());
    }

    pub fn add_landing_sequence(&mut self, wind_direction: f64) {
        let first_waypoint = get_landing_waypoint(wind_direction);
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

        self.add_waypoint(60, 52.282812822205244, 6.899070950501141);
        self.add_waypoint(30, 52.282666320797524, 6.898523816647838);

        self.add_special_waypoint(
            Some(0),
            MavCmd::MAV_CMD_NAV_LAND,
            [
                Some(0.0),
                Some(1.0),
                Some(0.0),
                None,
                Some(self.mission.plannedHomePosition[0]),
                Some(self.mission.plannedHomePosition[1]),
                Some(0.0),
            ],
        );
    }

    pub fn add_path(&mut self, path: &Vec<geo::Point>) {
        for point in path.iter() {
            self.add_waypoint(120, point.x(), point.y());
        }
    }

    /// Add goal position as loiter point.
    pub fn add_goal_position(&mut self, goal: &geo::Point) {
        self.add_special_waypoint(
            Some(120),
            MavCmd::MAV_CMD_NAV_LOITER_UNLIM,
            [
                Some(0.0),
                Some(0.0),
                Some(140.0),
                None,
                Some(goal.x()),
                Some(goal.y()),
                Some(120.0),
            ],
        );
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
}

/// Get last waypoint of take off sequence
pub fn get_take_off_waypoint(wind_direction: f64) -> geo::Point<f64> {
    let geod = Geodesic::wgs84();
    let (lat, lon) = geod.direct(
        52.282812822205244,
        6.899070950501141,
        adjust_wind_direction(wind_direction),
        210.0,
    );
    geo::Point::new(lat, lon)
}

/// Get first waypoint of landings sequence
pub fn get_landing_waypoint(wind_direction: f64) -> geo::Point<f64> {
    let landing_direction = if wind_direction <= 180.0 {
        wind_direction + 180.0
    } else {
        wind_direction - 180.0
    };
    let geod = Geodesic::wgs84();
    let (lat, lon) = geod.direct(
        52.282812822205244,
        6.899070950501141,
        adjust_wind_direction(landing_direction),
        210.0,
    );
    geo::Point::new(lat, lon)
}

fn adjust_wind_direction(wind_direction: f64) -> f64 {
    if wind_direction > 90.0 && wind_direction < 200.0 {
        if wind_direction < 180.0 {
            wind_direction + 180.0
        } else {
            wind_direction - 180.0
        }
    } else {
        wind_direction
    }
}
