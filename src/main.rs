pub mod dataStructs;

use std::fs::File;
use std::io::BufWriter;
use serde::Deserialize;

use crate::dataStructs::*;

#[derive(Debug, Deserialize)]
struct HourlyUnit {
    time: String,
    winddirection_10m: String,
    winddirection_80m: String,
    winddirection_120m: String,
}

#[derive(Debug, Deserialize)]
struct HourlyData {
    time: Vec<String>,
    winddirection_10m: Vec<i32>,
    winddirection_80m: Vec<i32>,
    winddirection_120m: Vec<i32>,
}

#[derive(Debug, Deserialize)]
struct WeatherData {
    latitude: f64,
    longitude: f64,
    generationtime_ms: f64,
    utc_offset_seconds: i32,
    timezone: String,
    timezone_abbreviation: String,
    elevation: f64,
    hourly_units: HourlyUnit,
    hourly: HourlyData,
}

fn main() {
    let mut plan = MavLinkPlan::new();

    // plan.mission.items[0].MISSION_ITEM_ID = Some(1);
    // plan.mission.items[0].doJumpId = Some(1);

    // Create a file to save the formatted JSON
    let file = File::create("test.plan").expect("Failed to create file");
    let writer = BufWriter::new(file);

    // Serialize and format the data with newlines and indentation
    serde_json::to_writer_pretty(writer, &plan).expect("Failed to write JSON data to file");

    println!("Data has been saved as 'test.json'");

    let response = reqwest::blocking::get("https://api.open-meteo.com/v1/forecast?latitude=52.29&longitude=6.9&hourly=winddirection_10m,winddirection_80m,winddirection_120m").unwrap();
    let test: WeatherData = serde_json::from_reader(response).unwrap();
    println!("{}", test.hourly_units.time);
}
