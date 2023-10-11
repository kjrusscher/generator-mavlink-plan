pub mod dataStructs;

use serde_json::ser::PrettyFormatter;
use serde::Serialize;
use std::fs::File;
use std::io::BufWriter;

use crate::dataStructs::*;

fn main() {
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
        polygons: [geo_fence_polygon],
        version: 2,
    };

    let rally_points = RallyPoints {
        points: vec![[52.282504959546564, 6.898488645574076, 0.0]],
        version: 2,
    };

    let item = MavLinkSimpleItem {
        AMSLAltAboveTerrain: 30,
        Altitude: 30,
        AltitudeMode: 1,
        MISSION_ITEM_ID: 1,
        autoContinue: true,
        command: 22,
        doJumpId: 1,
        frame: 3,
        params: [
            Some(15.0),
            Some(0.0),
            Some(0.0),
            None,
            Some(52.2825397),
            Some(6.8984103),
            Some(30.0),
        ],
        type_name: "SimpleItem".to_string(),
    };

    let missio_n = Mission {
        cruiseSpeed: 28,
        firmwareType: 12,
        globalPlanAltitudeMode: 1,
        hoverSpeed: 8,
        items: vec![item],
        plannedHomePosition: [52.2825397, 6.8984103, 40.44],
    };

    let plan: MavLinkPlan = MavLinkPlan {
        fileType: "Plan".to_string(),
        version: 1,
        geoFence: geo_fence,
        groundStation: "QGroundControl".to_string(),
        mission: missio_n,
        rallyPoints: rally_points,
    };

    // Create a file to save the formatted JSON
    let file = File::create("test.plan").expect("Failed to create file");
    let writer = BufWriter::new(file);

    // Serialize and format the data with newlines and indentation
    serde_json::to_writer_pretty(writer, &plan).expect("Failed to write JSON data to file");

    println!("Data has been saved as 'test.json'");
}
