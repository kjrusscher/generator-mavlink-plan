//! Mavlink Path Generator

pub mod astar_planner;
pub mod mav_link_plan;

use iced::widget::{
    button, column, container, pick_list, row, scrollable, text, vertical_space, Button, Column,
    Container, Space, Text,
};
use iced::{Alignment, Element, Length, Sandbox, Settings};
use notify_rust::Notification;
use serde::Deserialize;
use std::collections::HashMap;
use std::fs::File;
use std::io::BufWriter;
use std::time::Instant;

use crate::mav_link_plan::*;

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
    weather_data: Option<WeatherData>,
    wind_unit: Option<String>,
    data: HashMap<String, WindData>,
    wind_data: WindData,
    optimal_path: Vec<geo::Point>,
    pick_list_time: PickListTime,
    pop_up: PopUpInfo,
}

struct PopUpInfo {
    text: String,
    show: bool,
}

struct PickListTime {
    time_options: Vec<String>,
    selected_time: Option<String>,
}

/// Definitions for Iced
#[derive(Debug, Clone)]
enum Message {
    OptionSelected(String),
    SavePressed,
    AStarTest,
    UpdateWeatherInfo,
    PopUpPressed,
}

impl Sandbox for MavlinkPlanGenerator {
    type Message = Message;

    fn new() -> MavlinkPlanGenerator {
        let mut data = HashMap::new();
        let mut weather_info = None;
        let mut wind_unit = None;

        let pick_list_time = PickListTime {
            time_options: Vec::new(),
            selected_time: None,
        };

        let pop_up_info = PopUpInfo{
            text: "".to_string(),
            show: false,
        };

        MavlinkPlanGenerator {
            plan: None,
            data: data,
            weather_data: weather_info,
            wind_unit: wind_unit,
            wind_data: WindData {
                direction_10m: None,
                direction_80m: None,
                direction_120m: None,
            },
            optimal_path: Vec::new(),
            pick_list_time: pick_list_time,
            pop_up: pop_up_info,
        }
    }

    fn title(&self) -> String {
        String::from("Mavlink Plan Generator")
    }

    fn update(&mut self, message: Message) {
        match message {
            Message::OptionSelected(time) => {
                self.pick_list_time.selected_time = Some(time);
                self.wind_data = WindData {
                    direction_10m: self
                        .data
                        .get(&self.pick_list_time.selected_time.clone().unwrap())
                        .unwrap()
                        .direction_10m,
                    direction_80m: self
                        .data
                        .get(&self.pick_list_time.selected_time.clone().unwrap())
                        .unwrap()
                        .direction_80m,
                    direction_120m: self
                        .data
                        .get(&self.pick_list_time.selected_time.clone().unwrap())
                        .unwrap()
                        .direction_120m,
                };
            }
            Message::SavePressed => {
                if self.optimal_path.len() == 0 {
                    self.pop_up.text = "Plan eerst een route".to_string();
                    self.pop_up.show = true;
                } else if self.pick_list_time.selected_time.is_none() {
                    self.pop_up.text = "Selecteer een tijd".to_string();
                    self.pop_up.show = true;
                } else {
                    self.plan = Some(MavLinkPlan::new(
                        self.wind_data.direction_10m.unwrap(),
                        &self.optimal_path,
                    ));

                    let file_name = format!(
                        "vluchtplan_{}.plan",
                        self.pick_list_time.selected_time.as_ref().unwrap()
                    );

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
            Message::AStarTest => {
                println!("AStar pressed");
                let start_position = geo::Point::new(52.2825397, 6.8984103);
                let start_heading = 80.0;
                // goal ver weg
                // let goal_position = geo::Point::new(52.3825397, 6.9984103);
                // let goal_position = geo::Point::new(52.28458212,6.86716039);
                // let goal_position = geo::Point::new(52.28838126, 6.8706142);
                // let goal_position = geo::Point::new(52.2818941, 6.8786913);
                let goal_position = geo::Point::new(52.2738255, 6.8779212);
                let mut test_a_star_planner =
                    astar_planner::AStarPlanner::new(start_position, start_heading, goal_position)
                        .unwrap();
                let start = Instant::now();
                test_a_star_planner.calculate_path();
                let duration = start.elapsed();
                self.optimal_path = test_a_star_planner.get_optimal_path();
                // self.optimal_path = test_a_star_planner.get_all_points();
                println!("Route geplanned in {:.1?}.", duration);
            }
            Message::UpdateWeatherInfo => {
                let response = reqwest::blocking::get("https://api.open-meteo.com/v1/forecast?latitude=52.29&longitude=6.9&hourly=winddirection_10m,winddirection_80m,winddirection_120m");
                match response {
                    Ok(weather_response) => {
                        let weather_info: WeatherData;
                        weather_info = serde_json::from_reader(weather_response).unwrap();

                        self.wind_unit = Some(weather_info.hourly_units.winddirection_10m.clone());

                        for i in 0..weather_info.hourly.time.len() {
                            let wind_data = WindData {
                                direction_10m: Some(weather_info.hourly.winddirection_10m[i]),
                                direction_80m: Some(weather_info.hourly.winddirection_80m[i]),
                                direction_120m: Some(weather_info.hourly.winddirection_120m[i]),
                            };

                            self.data
                                .insert(weather_info.hourly.time[i].clone(), wind_data);
                        }
                        self.pick_list_time.time_options = weather_info.hourly.time.clone();
                        self.weather_data = Some(weather_info);
                    }
                    Err(error) => {
                        self.pop_up.text = "Kon geen weersinformatie ophalen. Waarschijnlijk geen internetverbinding".to_string();
                        self.pop_up.show = true;
                    }
                }
            }
            Message::PopUpPressed => {
                self.pop_up.show = false;
            }
        }
    }

    fn view(&self) -> Element<Message> {
        let picklist = pick_list(
            &self.pick_list_time.time_options,
            self.pick_list_time.selected_time.clone(),
            Message::OptionSelected,
        )
        .placeholder("Kies een tijd...");
        // let picklist = pick_list(
        //     &self.weather_data.hourly.time,
        //     self.selected_time.clone(),
        //     Message::OptionSelected,
        // )
        // .placeholder("Kies een tijd...");

        let start_position = geo::Point::new(52.2825397, 6.8984103);

        let start_location_text = text(format!("Drone positie:")).size(25);
        let start_location_gps = text(format!(
            "long: {:.2}, latt: {:.2}",
            start_position.x(),
            start_position.y() // self.plan.mission.plannedHomePosition[0], self.plan.mission.plannedHomePosition[1]
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
        let button_weather =
            Button::new("Update Weersinformatie").on_press(Message::UpdateWeatherInfo);

        let middle_column = column![
            vertical_space(60),
            text(format!("Windrichting op:")).size(25),
            wind_direction_10,
            wind_direction_80,
            wind_direction_120,
            button_weather,
            vertical_space(600)
        ]
        .width(Length::Fill)
        .align_items(Alignment::Center)
        .spacing(10);

        // let button = button("Opslaan").on_press(Message::SavePressed);
        // Create the button
        let button_save = Button::new("Opslaan").on_press(Message::SavePressed);
        let button_astar_test = Button::new("Plan Route").on_press((Message::AStarTest));

        let right_column = column![
            vertical_space(60),
            text(format!("Vluchtplan")).size(25),
            button_astar_test,
            button_save,
            vertical_space(600)
        ]
        .width(Length::Fill)
        .align_items(Alignment::Center)
        .spacing(10);

        let main_content = row![left_column, middle_column, right_column]
            .align_items(Alignment::Center)
            .spacing(20);

        let main_container: Container<'_, Message> = container(scrollable(main_content))
            .width(Length::Fill)
            .height(Length::Fill)
            .center_x()
            .center_y()
            .into();

        let mut popup_content = Column::new().push(main_container);

        if self.pop_up.show {
            let popup = Button::new(Text::new(&self.pop_up.text)).on_press(Message::PopUpPressed);
            let popup_container = Container::new(popup)
                .width(Length::Fill)
                .height(Length::Fill)
                .center_x()
                .center_y();

            // Overlay the popup over the main popup_content
            popup_content = popup_content
                .push(Space::with_height(Length::FillPortion(1)))
                .push(popup_container)
                .push(Space::with_height(Length::FillPortion(1)));
        }

        Container::new(popup_content)
            .width(Length::Fill)
            .height(Length::Fill)
            .into()
    }
}

pub fn main() -> iced::Result {
    let file = std::fs::read_to_string("Geofence_Fase_1_zonder_exclusion_zone.plan").unwrap();
    let plan: mav_link_plan::MavLinkPlan = serde_json::from_str(&file).unwrap();
    println!(
        "{}:{}: inclusion is {}.",
        file!(),
        line!(),
        plan.geoFence.polygons[0].inclusion
    );
    MavlinkPlanGenerator::run(Settings::default())
}
