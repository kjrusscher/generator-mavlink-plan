use geo;
use geographiclib_rs::{DirectGeodesic, Geodesic, InverseGeodesic};
use serde::Serialize;
use serde_repr::Serialize_repr;

// This enum is also in the mavlink crate. It's there under mavlink::common::MavCmd.
// Somehow enum to number does not serialize well for mavlink::common::MavComd, so we made our own.
#[derive(Serialize_repr)]
#[repr(u16)]
pub enum MavCmd {
    MAV_CMD_NAV_WAYPOINT = 16,
    MAV_CMD_NAV_LAND = 21,
    MAV_CMD_NAV_TAKEOFF = 22,
    MAV_CMD_DO_JUMP = 177,
    MAV_CMD_DO_VTOL_TRANSITION = 3000,
}

#[derive(Serialize)]
pub struct Circle {
    pub center: [f64; 2],
    pub radius: f64,
}

#[derive(Serialize)]
pub struct GeoFenceCircle {
    pub circle: Circle,
    pub inclusion: bool,
    pub version: i32,
}

#[derive(Serialize)]
pub struct GeoFencePolygon {
    pub inclusion: bool,
    pub polygon: Vec<[f64; 2]>,
    pub version: i32,
}

#[derive(Serialize)]
pub struct GeoFence {
    pub circles: Vec<GeoFenceCircle>,
    pub polygons: Vec<GeoFencePolygon>,
    pub version: i32,
}

#[derive(Serialize)]
pub struct RallyPoints {
    pub points: Vec<[f64; 3]>,
    pub version: i32,
}

#[derive(Serialize)]
pub struct MavLinkSimpleItem {
    pub AMSLAltAboveTerrain: Option<i32>,
    pub Altitude: Option<i32>,
    pub AltitudeMode: i32,
    pub MISSION_ITEM_ID: Option<i32>,
    pub autoContinue: bool,
    pub command: MavCmd,
    pub doJumpId: Option<i32>,
    pub frame: i32,
    pub params: [Option<f64>; 7],
    #[serde(rename = "type")]
    pub type_name: String,
}

#[derive(Serialize)]
pub struct Mission {
    pub cruiseSpeed: i32,
    pub firmwareType: i32,
    pub globalPlanAltitudeMode: i32,
    pub hoverSpeed: i32,
    pub items: Vec<MavLinkSimpleItem>,
    pub plannedHomePosition: [f64; 3],
}

#[derive(Serialize)]
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
        let geo_fence_polygon = GeoFencePolygon {
            inclusion: true,
            version: 1,
            polygon: vec![
                [52.282599366629, 6.864644466619154],
                [52.29132654581896, 6.902744730087136],
                [52.28661072881356, 6.915918601884101],
                [52.2830192805577, 6.91078335454705],
                [52.27591560765218, 6.893539021753156],
                [52.27399925141548, 6.874775277212848],
            ],
        };

        let geo_fence = GeoFence {
            circles: vec![],
            polygons: vec![geo_fence_polygon],
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

        let plan: MavLinkPlan = MavLinkPlan {
            fileType: String::from("Plan"),
            version: 1,
            geoFence: geo_fence,
            groundStation: String::from("QGroundControl"),
            mission: mission,
            rallyPoints: rally_points,
        };

        return plan;
    }
}

impl MavLinkPlan {
    pub fn new(wind_direction_10m: i32, wind_direction_80m: i32) -> MavLinkPlan {
        let mut plan = MavLinkPlan::default();
        let geod = Geodesic::wgs84();

        let direction_take_off = f64::from(wind_direction_10m);
        let direction_landing: f64;
        if direction_take_off < 180.0 {
            direction_landing = f64::from(wind_direction_10m) + 180.0;
        } else {
            direction_landing = f64::from(wind_direction_10m) - 180.0;
        }
        
        let position_pilot = geo::Point::new(52.282432, 6.898313);
        let position_home_drone = geo::Point::new(52.282578, 6.898316);
        // Calculate the azimuth from p1 to p2.
        let (azi1, _, _) = geod.inverse(position_pilot.x(), position_pilot.y(), position_home_drone.x(), position_home_drone.y());

        println!("azi1: {} degrees", azi1);

        let (mut lat, mut lon) =
            geod.direct(position_home_drone.x(), position_home_drone.y(), 30.0, 35.0);
        let vtol_transition = geo::Point::new(lat, lon);

        plan.add_special_waypoint(
            Some(30),
            MavCmd::MAV_CMD_NAV_TAKEOFF,
            [
                Some(15.0),
                Some(0.0),
                Some(0.0),
                Some(direction_take_off),
                Some(position_home_drone.x()),
                Some(position_home_drone.y()),
                Some(30.0),
            ],
        );

        plan.add_waypoint(60, vtol_transition.x(), vtol_transition.y());

        plan.add_special_waypoint(
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

        (lat, lon) = geod.direct(
            vtol_transition.x(),
            vtol_transition.y(),
            direction_take_off,
            210.0,
        );
        let mut current_point = geo::Point::new(lat, lon);
        plan.add_waypoint(60, current_point.x(), current_point.y());

        // plan.add_waypoint(80, 52.28528971356268, 6.906843683019673);
        // plan.add_waypoint(85, 52.28619508983822, 6.907700944308999);
        // plan.add_waypoint(90, 52.28700933019922, 6.906646500436068);
        // plan.add_waypoint(95, 52.28691788742173, 6.904588489118652);
        // plan.add_waypoint(120, 52.27967402543756, 6.889267721564778);
        // plan.add_waypoint(120, 52.27892018775579, 6.878703588517652);
        // plan.add_waypoint(120, 52.27938806255716, 6.876429240060219);
        // plan.add_waypoint(120, 52.280655681513785, 6.87645870522141);
        // plan.add_waypoint(120, 52.28111306136718, 6.878545693531606);
        // plan.add_waypoint(120, 52.281507456033765, 6.897465752194307);
        // plan.add_waypoint(120, 52.28521598112396, 6.907070280875956);
        // plan.add_waypoint(120, 52.28623948195102, 6.908008241913279);
        // plan.add_waypoint(120, 52.28716558760136, 6.906745978228713);
        // plan.add_waypoint(120, 52.286970052792455, 6.9044260509882065);

        // plan.add_special_waypoint(
        //     None,
        //     MavCmd::MAV_CMD_DO_JUMP,
        //     [
        //         Some(9.0),
        //         Some(10.0),
        //         Some(0.0),
        //         Some(0.0),
        //         Some(0.0),
        //         Some(0.0),
        //         Some(0.0),
        //     ],
        // );

        // plan.add_waypoint(80, 52.28381947097879, 6.901992010013913);

        (lat, lon) = geod.direct(
            vtol_transition.x(),
            vtol_transition.y(),
            direction_landing,
            210.0,
        );
        current_point = geo::Point::new(lat, lon);
        plan.add_waypoint(60, current_point.x(), current_point.y());

        plan.add_waypoint(60, vtol_transition.x(), vtol_transition.y());

        plan.add_special_waypoint(
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

        plan.add_special_waypoint(
            Some(0),
            MavCmd::MAV_CMD_NAV_LAND,
            [
                Some(0.0),
                Some(1.0),
                Some(0.0),
                None,
                Some(position_home_drone.x()),
                Some(position_home_drone.y()),
                Some(0.0),
            ],
        );

        return plan;
    }

    pub fn add_waypoint(&mut self, height: i32, latitude: f64, longitude: f64) {
        let mut waypoint = MavLinkSimpleItem::default();
        let id: i32 = (self.mission.items.len() as i32) + 1;
        waypoint.AMSLAltAboveTerrain = Some(height);
        waypoint.Altitude = Some(height);
        waypoint.MISSION_ITEM_ID = Some(id);
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

    pub fn add_special_waypoint(
        &mut self,
        height: Option<i32>,
        command: MavCmd,
        params: [Option<f64>; 7],
    ) {
        let mut waypoint = MavLinkSimpleItem::default();
        let id: i32 = (self.mission.items.len() as i32) + 1;
        waypoint.AMSLAltAboveTerrain = height;
        waypoint.Altitude = height;
        waypoint.MISSION_ITEM_ID = Some(id);
        waypoint.command = command;
        waypoint.doJumpId = Some(id);
        waypoint.params = params;

        self.mission.items.push(waypoint);
    }
}
