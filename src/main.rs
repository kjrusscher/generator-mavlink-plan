//! Mavlink Path Generator

pub mod astar_planner;
pub mod mav_link_plan;

use iced::widget::{
    column, container, pick_list, row, scrollable, text, vertical_space, Button, Column, Container,
    Space, Text, TextInput,
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

struct AppWeatherInfo {
    weather_data: Option<WeatherData>,
    wind_unit: Option<String>,
    data: HashMap<String, WindData>,
    wind_data: WindData,
}

struct AppPopUpInfo {
    text: String,
    show: bool,
}

struct AppPickListTime {
    time_options: Vec<String>,
    selected_time: Option<String>,
    time_picked: bool,
}

struct AppPathInfo {
    drone_position: geo::Point,
    goal_position: geo::Point,
    optimal_path_from_take_off_to_goal: Vec<geo::Point>,
    optimal_path_from_goal_to_landing: Vec<geo::Point>,
    path_planned: bool,
}

/// Stores state information for application
struct MavlinkPlanGenerator {
    plan: Option<MavLinkPlan>,
    path_info: AppPathInfo,
    weather_info: AppWeatherInfo,
    pick_list_time: AppPickListTime,
    pop_up: AppPopUpInfo,
    file_name: String,
}

/// Definitions for Iced
#[derive(Debug, Clone)]
enum Message {
    OptionSelected(String),
    SavePressed,
    PlanRoute,
    UpdateWeatherInfo,
    PopUpPressed,
    InputDroneLongitudeChanged(String),
    InputDroneLatitudeChanged(String),
    InputGoalLongitudeChanged(String),
    InputGoalLatitudeChanged(String),
    InputFileName(String),
    LoadPressed,
}

impl Sandbox for MavlinkPlanGenerator {
    type Message = Message;

    fn new() -> MavlinkPlanGenerator {
        let data = HashMap::new();
        let weather_info = None;
        let wind_unit = None;

        let pick_list_time = AppPickListTime {
            time_options: Vec::new(),
            selected_time: None,
            time_picked: false,
        };

        let pop_up_info = AppPopUpInfo {
            text: "".to_string(),
            show: false,
        };

        let weather_info = AppWeatherInfo {
            data: data,
            weather_data: weather_info,
            wind_unit: wind_unit,
            wind_data: WindData {
                direction_10m: None,
                direction_80m: None,
                direction_120m: None,
            },
        };

        let geo_info = AppPathInfo {
            drone_position: geo::Point::new(52.282538406253, 6.898364382855505),
            goal_position: geo::Point::new(52.2878797, 6.8706270),
            optimal_path_from_take_off_to_goal: Vec::new(),
            optimal_path_from_goal_to_landing: Vec::new(),
            path_planned: false,
        };

        MavlinkPlanGenerator {
            plan: None,
            weather_info: weather_info,
            path_info: geo_info,
            pick_list_time: pick_list_time,
            pop_up: pop_up_info,
            file_name: "Geofence Fase I met exclusion zone KJR.plan".to_string(),
        }
    }

    fn title(&self) -> String {
        String::from("Mavlink Plan Generator")
    }

    fn update(&mut self, message: Message) {
        match message {
            Message::OptionSelected(time) => {
                self.pick_list_time.selected_time = Some(time);
                self.weather_info.wind_data = WindData {
                    direction_10m: self
                        .weather_info
                        .data
                        .get(&self.pick_list_time.selected_time.clone().unwrap())
                        .unwrap()
                        .direction_10m,
                    direction_80m: self
                        .weather_info
                        .data
                        .get(&self.pick_list_time.selected_time.clone().unwrap())
                        .unwrap()
                        .direction_80m,
                    direction_120m: self
                        .weather_info
                        .data
                        .get(&self.pick_list_time.selected_time.clone().unwrap())
                        .unwrap()
                        .direction_120m,
                };
                self.pick_list_time.time_picked = true;
            }
            Message::SavePressed => {
                if self.pick_list_time.selected_time.is_none() {
                    self.pop_up.text = "Selecteer een tijd".to_string();
                    self.pop_up.show = true;
                } else if self.path_info.optimal_path_from_take_off_to_goal.len() == 0
                    || self.path_info.optimal_path_from_goal_to_landing.len() == 0
                {
                    self.pop_up.text = "Plan eerst een route".to_string();
                    self.pop_up.show = true;
                } else {
                    let file_name = format!(
                        "vluchtplan_{}.plan",
                        self.pick_list_time.selected_time.as_ref().unwrap()
                    );
                    self.save_plan_to_file(&file_name);

                    Notification::new()
                        .summary("Bestand Opgeslagen")
                        .body(&file_name)
                        .show()
                        .unwrap();
                }
            }
            Message::PlanRoute => {
                let start_position = mav_link_plan::get_take_off_waypoint(f64::from(
                    self.weather_info.wind_data.direction_10m.unwrap(),
                ));
                let start_heading = f64::from(self.weather_info.wind_data.direction_10m.unwrap());
                let end_position = mav_link_plan::get_landing_waypoint(f64::from(
                    self.weather_info.wind_data.direction_10m.unwrap(),
                ));
                let end_heading = f64::from(self.weather_info.wind_data.direction_10m.unwrap());
                let test_a_star_planner = astar_planner::AStarPlanner::new(
                    start_position,
                    start_heading,
                    self.path_info.goal_position,
                    end_position,
                    end_heading,
                );
                
                match test_a_star_planner {
                    Ok(mut a_star_planner) => {
                        if let Some(plan) = self.plan.as_ref() {
                            a_star_planner.add_geo_fences(&plan.geoFence);
                        }
                        let start = Instant::now();
                        self.path_info.optimal_path_from_take_off_to_goal =
                            a_star_planner.get_optimal_path_to_goal();
                        self.path_info.optimal_path_from_goal_to_landing =
                            a_star_planner.get_optimal_path_from_goal();
                        let duration = start.elapsed();
                        // self.path_info.optimal_path = a_star_planner.get_all_points();
                        println!("Route geplanned in {:.1?}.", duration);
                        self.path_info.path_planned = true;
                    }
                    Err(message) => {
                        self.pop_up.text = message + ". Pas deze waarde aan, aub.";
                        self.pop_up.show = true;
                    }
                }
            }
            Message::UpdateWeatherInfo => {
                let response = reqwest::blocking::get("https://api.open-meteo.com/v1/forecast?latitude=52.29&longitude=6.9&hourly=winddirection_10m,winddirection_80m,winddirection_120m");
                match response {
                    Ok(weather_response) => {
                        let weather_info: WeatherData;
                        weather_info = serde_json::from_reader(weather_response).unwrap();

                        self.weather_info.wind_unit =
                            Some(weather_info.hourly_units.winddirection_10m.clone());

                        for i in 0..weather_info.hourly.time.len() {
                            let wind_data = WindData {
                                direction_10m: Some(weather_info.hourly.winddirection_10m[i]),
                                direction_80m: Some(weather_info.hourly.winddirection_80m[i]),
                                direction_120m: Some(weather_info.hourly.winddirection_120m[i]),
                            };

                            self.weather_info
                                .data
                                .insert(weather_info.hourly.time[i].clone(), wind_data);
                        }
                        self.pick_list_time.time_options = weather_info.hourly.time.clone();
                        if self.pick_list_time.selected_time.is_none() {
                            self.pick_list_time.selected_time =
                                Some(self.pick_list_time.time_options[0].clone());
                        }
                        self.pick_list_time.time_picked = true;
                        self.weather_info.weather_data = Some(weather_info);
                        self.weather_info.wind_data = WindData {
                            direction_10m: self
                                .weather_info
                                .data
                                .get(&self.pick_list_time.selected_time.clone().unwrap())
                                .unwrap()
                                .direction_10m,
                            direction_80m: self
                                .weather_info
                                .data
                                .get(&self.pick_list_time.selected_time.clone().unwrap())
                                .unwrap()
                                .direction_80m,
                            direction_120m: self
                                .weather_info
                                .data
                                .get(&self.pick_list_time.selected_time.clone().unwrap())
                                .unwrap()
                                .direction_120m,
                        };
                    }
                    Err(_) => {
                        self.pop_up.text = "Kon geen weersinformatie ophalen. Waarschijnlijk geen internetverbinding".to_string();
                        self.pop_up.show = true;
                    }
                }
            }
            Message::PopUpPressed => {
                self.pop_up.show = false;
            }
            Message::InputDroneLongitudeChanged(string_value) => {
                if let Ok(value) = string_value.parse::<f64>() {
                    self.path_info.drone_position.set_x(value);
                }
            }
            Message::InputDroneLatitudeChanged(string_value) => {
                if let Ok(value) = string_value.parse::<f64>() {
                    self.path_info.drone_position.set_x(value);
                }
            }
            Message::InputGoalLongitudeChanged(string_value) => {
                if let Ok(value) = string_value.parse::<f64>() {
                    self.path_info.goal_position.set_x(value);
                }
            }
            Message::InputGoalLatitudeChanged(string_value) => {
                if let Ok(value) = string_value.parse::<f64>() {
                    self.path_info.goal_position.set_y(value);
                }
            }
            Message::InputFileName(string_value) => {
                self.file_name = string_value;
            }
            Message::LoadPressed => {
                let file = std::fs::read_to_string(&self.file_name).unwrap();
                self.plan = serde_json::from_str(&file).unwrap();
                println!(
                    "{}:{}: inclusion is {}.",
                    file!(),
                    line!(),
                    self.plan.as_ref().unwrap().geoFence.polygons[0].inclusion
                );
            }
        }
    }

    fn view(&self) -> Element<Message> {
        let file_name_text = text(format!("Bestandsnaam")).size(25);
        let file_name_input = TextInput::new("File Name", &self.file_name)
            .on_input(Message::InputFileName)
            .width(Length::Fill);
        let button_load = Button::new("Laad .plan bestand").on_press(Message::LoadPressed);

        let start_location_text = text(format!("Drone")).size(25);
        let drone_longitude = TextInput::new(
            "Longitude",
            &self.path_info.drone_position.x().to_string(),
        )
        .on_input(Message::InputDroneLongitudeChanged)
        .width(Length::Fixed(190.0));
        let drone_latitude = TextInput::new(
            "Latitude ",
            &self.path_info.drone_position.y().to_string(),
        )
        .on_input(Message::InputDroneLatitudeChanged)
        .width(Length::Fixed(190.0));

        let input_longitude = TextInput::new(
            "Longitude",
            &self.path_info.goal_position.x().to_string(),
        )
        .on_input(Message::InputGoalLongitudeChanged)
        .width(Length::Fixed(190.0));
        let input_latitude = TextInput::new(
            "Latitude ",
            &self.path_info.goal_position.y().to_string(),
        )
        .on_input(Message::InputGoalLatitudeChanged)
        .width(Length::Fixed(190.0));

        let left_column = column![
            vertical_space(20),
            file_name_text,
            file_name_input,
            button_load,
            vertical_space(20),
            start_location_text,
            row![
                column!["Longitude:", vertical_space(10), "Latitude:"].align_items(Alignment::End),
                column![drone_longitude, drone_latitude]
            ]
            .align_items(Alignment::Center),
            vertical_space(30),
            text(format!("Doel")).size(25),
            row![
                column!["Longitude:", vertical_space(10), "Latitude: "].align_items(Alignment::End),
                column![input_longitude, input_latitude]
            ]
            .align_items(Alignment::Center),
            vertical_space(30),
        ]
        .width(Length::Fill)
        .align_items(Alignment::Center)
        .spacing(10);

        let wind_direction_10 = text(format!(
            " 10 meter: {}{}",
            self.weather_info
                .wind_data
                .direction_10m
                .unwrap_or(0)
                .to_string(),
            self.weather_info
                .wind_unit
                .as_ref()
                .unwrap_or(&"°".to_string())
        ))
        .size(20);
        let wind_direction_80 = text(format!(
            " 80 meter: {}{}",
            self.weather_info
                .wind_data
                .direction_80m
                .unwrap_or(0)
                .to_string(),
            self.weather_info
                .wind_unit
                .as_ref()
                .unwrap_or(&"°".to_string())
        ))
        .size(20);
        let wind_direction_120 = text(format!(
            "120 meter: {}{}",
            self.weather_info
                .wind_data
                .direction_120m
                .unwrap_or(0)
                .to_string(),
            self.weather_info
                .wind_unit
                .as_ref()
                .unwrap_or(&"°".to_string())
        ))
        .size(20);
        let button_weather = Button::new("Update").on_press(Message::UpdateWeatherInfo);

        let picklist = pick_list(
            &self.pick_list_time.time_options,
            self.pick_list_time.selected_time.clone(),
            Message::OptionSelected,
        )
        .placeholder("Kies een tijd...");

        let middle_column = column![
            vertical_space(20),
            text(format!("Windrichting")).size(25),
            wind_direction_10,
            wind_direction_80,
            wind_direction_120,
            button_weather,
            vertical_space(20),
            text(format!("Wanneer wil je vliegen?")).size(25),
            picklist,
            vertical_space(20),
        ]
        .width(Length::Fill)
        .align_items(Alignment::Center)
        .spacing(10);

        let button_save_enabled: Option<Message>;
        if self.pick_list_time.time_picked && self.path_info.path_planned {
            button_save_enabled = Some(Message::SavePressed);
        } else {
            button_save_enabled = None;
        }
        let button_save = Button::new("Opslaan").on_press_maybe(button_save_enabled);
        let button_astar_enabled: Option<Message>;
        if self.pick_list_time.time_picked {
            button_astar_enabled = Some(Message::PlanRoute);
        } else {
            button_astar_enabled = None;
        }
        let button_astar = Button::new("Plan Route").on_press_maybe(button_astar_enabled);

        let right_column = column![
            vertical_space(20),
            text(format!("Vluchtplan")).size(25),
            button_astar,
            button_save,
            vertical_space(20)
        ]
        .width(Length::Fill)
        .align_items(Alignment::Center)
        .spacing(10);

        let main_content =
            row![left_column, middle_column, right_column].align_items(Alignment::Start);

        let main_container: Container<'_, Message> = container(scrollable(main_content))
            .width(Length::Shrink)
            .height(Length::Shrink)
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
                .push(Space::with_height(Length::Fill))
                .push(popup_container)
                .push(Space::with_height(Length::Fill));
        }

        Container::new(popup_content)
            .width(Length::Fill)
            .height(Length::Fill)
            .into()
    }
}

impl MavlinkPlanGenerator {
    fn save_plan_to_file(&mut self, file_name: &String) {
        if let Some(plan) = self.plan.as_mut() {
            plan.add_take_off_sequence(f64::from(self.weather_info.wind_data.direction_10m.unwrap()));
            plan.add_path(&self.path_info.optimal_path_from_take_off_to_goal);
            plan.add_goal_position(&self.path_info.goal_position);
            plan.add_path(&self.path_info.optimal_path_from_goal_to_landing);
            plan.add_landing_sequence(f64::from(self.weather_info.wind_data.direction_10m.unwrap()));

            // Create a file to save the formatted JSON
            let file = File::create(&file_name).expect("Failed to create file");
            let writer = BufWriter::new(file);

            // Serialize and format the data with newlines and indentation
            serde_json::to_writer_pretty(writer, &plan).expect("Failed to write JSON data to file");
        }
    }
}

pub fn main() -> iced::Result {
    MavlinkPlanGenerator::run(Settings::default())
}
