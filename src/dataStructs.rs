use serde::Serialize;

#[derive(Serialize)]
pub struct GeoFencePolygon {
    pub inclusion: bool,
    pub polygon: Vec<[f64; 2]>,
    pub version: i32,
}

#[derive(Serialize)]
pub struct GeoFence {
    pub circles: Vec<i32>,
    pub polygons: GeoFencePolygon,
    pub version: i32,
}

#[derive(Serialize)]
pub struct RallyPoints {
    pub points: Vec<[f64; 3]>,
    pub version: i32,
}

#[derive(Serialize)]
pub struct MavLinkSimpleItem {
    pub AMSLAltAboveTerrain: i32,
    pub Altitude: i32,
    pub AltitudeMode: i32,
    pub autoContinue: bool,
    pub command: i32,
    pub doJumpId: i32,
    pub frame: i32,
    pub params: [f64; 7],
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
    pub filetype: String,
    // pub geoFence: GeoFence,
    pub groundStation: String,
    // pub mission: Mission,
    // pub rallyPoints: RallyPoints,
    pub version: u8,
}
