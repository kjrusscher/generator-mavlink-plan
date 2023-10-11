pub mod dataStructs;

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

    

    let plan: MavLinkPlan = MavLinkPlan {
        filetype: "Plan".to_string(),
        version: 1,
        // geoFence: dataStructs::GeoFence,
        groundStation: "QGroundControl".to_string(),
        // mission: null(),
        // rallyPoints: null()
    };

    // Create a file to save the formatted JSON
    let file = File::create("test.plan").expect("Failed to create file");
    let writer = BufWriter::new(file);

    // Serialize and format the data with newlines and indentation
    serde_json::to_writer_pretty(writer, &plan).expect("Failed to write JSON data to file");

    println!("Data has been saved as 'test.json'");
}
