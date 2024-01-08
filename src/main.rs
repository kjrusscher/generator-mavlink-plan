//! Mavlink Path Generator

// pub mod astar_planner;
pub mod data_structs;

use iced::widget::{button, column, container, pick_list, row, scrollable, text, vertical_space};
use iced::{Alignment, Element, Length, Sandbox, Settings};
use notify_rust::Notification;
use serde::Deserialize;
use std::collections::HashMap;
use std::fs::File;
use std::io::BufWriter;

use crate::data_structs::*;

/// Wind directions at specific time
#[derive(Debug, Deserialize, Clone)]
#[allow(dead_code)] // Field are read from json, but not used.
struct HourlyUnit {
    time: String,
    winddirection_10m: String,
    winddirection_80m: String,
    winddirection_120m: String,
}

/// Wind directions for range of times
#[derive(Debug, Deserialize, Clone)]
struct HourlyData {
    time: Vec<String>,
    winddirection_10m: Vec<i32>,
    winddirection_80m: Vec<i32>,
    winddirection_120m: Vec<i32>,
}

/// For reading in weather data from open-meteo
#[derive(Debug, Deserialize, Clone)]
#[allow(dead_code)] // Field are read from json, but not used.
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

/// Wind data options
#[derive(Debug, Clone)]
struct WindData {
    direction_10m: Option<i32>,
    direction_80m: Option<i32>,
    direction_120m: Option<i32>,
}

/// Stores state information for application
struct MavlinkPlanGenerator {
    plan: Option<MavLinkPlan>,
    weather_data: WeatherData,
    wind_unit: Option<String>,
    data: HashMap<String, WindData>,
    selected_time: Option<String>,
    wind_data: WindData,
}

/// Definitions for Iced
#[derive(Debug, Clone)]
enum Message {
    OptionSelected(String),
    SavePressed,
}

impl Sandbox for MavlinkPlanGenerator {
    type Message = Message;

    fn new() -> MavlinkPlanGenerator {
        let response = reqwest::blocking::get("https://api.open-meteo.com/v1/forecast?latitude=52.29&longitude=6.9&hourly=winddirection_10m,winddirection_80m,winddirection_120m");
        let mut data = HashMap::new();
        let mut weather_info: WeatherData;
        let mut wind_unit = None;
        match response {
            Ok(weather_response) => {
                weather_info = serde_json::from_reader(weather_response).unwrap();

                wind_unit = Some(weather_info.hourly_units.winddirection_10m.clone());

                for i in 0..weather_info.hourly.time.len() {
                    let wind_data = WindData {
                        direction_10m: Some(weather_info.hourly.winddirection_10m[i]),
                        direction_80m: Some(weather_info.hourly.winddirection_80m[i]),
                        direction_120m: Some(weather_info.hourly.winddirection_120m[i]),
                    };

                    data.insert(weather_info.hourly.time[i].clone(), wind_data);
                }
            }
            Err(error) => { 
                panic!("Kon weersinformatie niet ophalen, waarschijnlijk geen internet verbinding. {}", error);
            }
        }

        MavlinkPlanGenerator {
            plan: None,
            data: data,
            weather_data: weather_info,
            wind_unit: wind_unit,
            selected_time: None,
            wind_data: WindData {
                direction_10m: None,
                direction_80m: None,
                direction_120m: None,
            },
        }
    }

    fn title(&self) -> String {
        String::from("Mavlink Plan Generator")
    }

    fn update(&mut self, message: Message) {
        match message {
            Message::OptionSelected(time) => {
                self.selected_time = Some(time);
                self.wind_data = WindData {
                    direction_10m: self
                        .data
                        .get(&self.selected_time.clone().unwrap())
                        .unwrap()
                        .direction_10m,
                    direction_80m: self
                        .data
                        .get(&self.selected_time.clone().unwrap())
                        .unwrap()
                        .direction_80m,
                    direction_120m: self
                        .data
                        .get(&self.selected_time.clone().unwrap())
                        .unwrap()
                        .direction_120m,
                };
                self.plan = Some(MavLinkPlan::new(
                    self.wind_data.direction_10m.unwrap(),
                    self.wind_data.direction_80m.unwrap(),
                ));
            }
            Message::SavePressed => {
                if self.selected_time.is_some() {
                    let file_name =
                        format!("vluchtplan_{}.plan", self.selected_time.as_ref().unwrap());

                    // Create a file to save the formatted JSON
                    let file = File::create(&file_name).expect("Failed to create file");
                    let writer = BufWriter::new(file);

                    // Serialize and format the data with newlines and indentation
                    serde_json::to_writer_pretty(writer, &self.plan)
                        .expect("Failed to write JSON data to file");

                    Notification::new()
                        .summary("Bestand Opgeslagen")
                        .body(&file_name)
                        .show()
                        .unwrap();
                }
            }
        }
    }

    fn view(&self) -> Element<Message> {
        let picklist = pick_list(
            &self.weather_data.hourly.time,
            self.selected_time.clone(),
            Message::OptionSelected,
        )
        .placeholder("Kies een tijd...");

        let start_location_text = text(format!("Drone positie:")).size(25);
        let start_location_gps = text(format!(
            "long: {:.2}, latt: {:.2}",
            52.2825397,
            6.8984103 // self.plan.mission.plannedHomePosition[0], self.plan.mission.plannedHomePosition[1]
        ))
        .size(20);

        let left_column = column![
            vertical_space(60),
            start_location_text,
            start_location_gps,
            vertical_space(30),
            text(format!("Wanneer wil je vliegen?")).size(25),
            picklist,
            vertical_space(600),
        ]
        .width(Length::Fill)
        .align_items(Alignment::Center)
        .spacing(10);

        let wind_direction_10 = text(format!(
            " 10 meter: {}{}",
            self.wind_data.direction_10m.unwrap_or(0).to_string(),
            self.wind_unit.as_ref().unwrap_or(&"°".to_string())
        ))
        .size(20);
        let wind_direction_80 = text(format!(
            " 80 meter: {}{}",
            self.wind_data.direction_80m.unwrap_or(0).to_string(),
            self.wind_unit.as_ref().unwrap_or(&"°".to_string())
        ))
        .size(20);
        let wind_direction_120 = text(format!(
            "120 meter: {}{}",
            self.wind_data.direction_120m.unwrap_or(0).to_string(),
            self.wind_unit.as_ref().unwrap_or(&"°".to_string())
        ))
        .size(20);

        let middle_column = column![
            vertical_space(60),
            text(format!("Windrichting op:")).size(25),
            wind_direction_10,
            wind_direction_80,
            wind_direction_120,
            vertical_space(600)
        ]
        .width(Length::Fill)
        .align_items(Alignment::Center)
        .spacing(10);

        let button = button("Opslaan").on_press(Message::SavePressed);

        let right_column = column![
            vertical_space(60),
            text(format!("Vluchtplan")).size(25),
            button,
            vertical_space(600)
        ]
        .width(Length::Fill)
        .align_items(Alignment::Center)
        .spacing(10);

        let content = row![left_column, middle_column, right_column]
            .align_items(Alignment::Center)
            .spacing(20);

        container(scrollable(content))
            .width(Length::Fill)
            .height(Length::Fill)
            .center_x()
            .center_y()
            .into()
    }
}

pub fn main() -> iced::Result {
    let file = std::fs::read_to_string("Geofence_Fase_1_zonder_exclusion_zone.plan").unwrap();
    let plan: data_structs::MavLinkPlan = serde_json::from_str(&file).unwrap();
    println!("{}:{}: inclusion is {}.",file!(), line!(), plan.geoFence.polygons[0].inclusion);
    MavlinkPlanGenerator::run(Settings::default())
}
