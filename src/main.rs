//! Mavlink Path Generator

pub mod astar_planner;
pub mod mav_link_plan;
use crate::mav_link_plan::*;
// use astar_planner::planning_waypoints::{GeospatialPose, Node};
use astar_planner::AStarPlannerBuilder;
use iced::widget::{
    checkbox, column, container, pick_list, row, scrollable, text, vertical_space, Button, Column,
    Container, Space, Text, TextInput,
};
use iced::{Alignment, Element, Length, Sandbox, Settings};
use notify_rust::Notification;
use serde::Deserialize;
use std::fs::File;
use std::io::BufWriter;
use std::time::Instant;

#[derive(Deserialize)]
struct WeatherData {
    hourly_units: HourlyUnits,
    hourly: Hourly,
}

#[derive(Deserialize)]
struct HourlyUnits {
    precipitation_probability: String,
    precipitation: String,
    wind_direction_80m: String,
}

#[derive(Deserialize)]
struct Hourly {
    time: Vec<String>,
    precipitation_probability: Vec<i32>,
    precipitation: Vec<f64>,
    wind_speed_120m: Vec<f64>,
    wind_speed_10m: Vec<f64>,
    wind_direction_80m: Vec<i32>,
}

struct AppWeatherInfo {
    weather_data: Option<WeatherData>,
    selected_time: Option<String>,
    selected_time_index: Option<usize>,
}

struct AppPopUpInfo {
    text: String,
    show: bool,
}

struct AppPlannerInfo {
    drone_position: geo::Point,
    goal_position: Option<geo::Point>,
    optimal_path_from_take_off_to_goal: Vec<geo::Point>,
    optimal_path_from_goal_to_landing: Vec<geo::Point>,
}

/// Stores state information for application
struct MavlinkPlanGenerator {
    plan_drone_and_goal: Option<MavLinkPlan>,
    file_name_drone_and_goal: Option<String>,
    plan_obstacles: Option<MavLinkPlan>,
    file_name_obstacles: Option<String>,
    path_info: AppPlannerInfo,
    weather_info: AppWeatherInfo,
    pop_up: AppPopUpInfo,
    obstacles_from_drone_and_goal_file: bool,
}

/// Definitions for Iced
#[derive(Debug, Clone)]
enum Message {
    OptionSelectedWeather(String),
    SavePressed,
    PlanRoute,
    ButtonUpdateWeatherInfo,
    PopUpPressed,
    InputDroneLongitudeChanged(String),
    InputDroneLatitudeChanged(String),
    InputGoalLongitudeChanged(String),
    InputGoalLatitudeChanged(String),
    OptionFileNameDroneAndGoal(String),
    OptionFileNameObstacles(String),
    CheckboxObstacles(bool),
}

impl Sandbox for MavlinkPlanGenerator {
    type Message = Message;

    fn new() -> MavlinkPlanGenerator {
        let pop_up_info = AppPopUpInfo {
            text: "".to_string(),
            show: false,
        };

        let weather_info = AppWeatherInfo {
            selected_time: None,
            selected_time_index: None,
            weather_data: None,
        };

        let geo_info = AppPlannerInfo {
            drone_position: geo::Point::new(52.2825397, 6.8984103),
            goal_position: None,
            optimal_path_from_take_off_to_goal: Vec::new(),
            optimal_path_from_goal_to_landing: Vec::new(),
        };

        MavlinkPlanGenerator {
            plan_drone_and_goal: None,
            file_name_drone_and_goal: None,
            plan_obstacles: None,
            file_name_obstacles: None,
            weather_info: weather_info,
            path_info: geo_info,
            pop_up: pop_up_info,
            obstacles_from_drone_and_goal_file: false,
        }
    }

    fn title(&self) -> String {
        String::from("Mavlink Plan Generator")
    }

    fn update(&mut self, message: Message) {
        match message {
            Message::OptionFileNameDroneAndGoal(string_value) => {
                self.file_name_drone_and_goal = Some(string_value);
                let file =
                    std::fs::read_to_string(&self.file_name_drone_and_goal.as_ref().unwrap())
                        .unwrap();
                self.plan_drone_and_goal = serde_json::from_str(&file).unwrap();
                if let Some(plan) = self.plan_drone_and_goal.as_ref() {
                    self.path_info
                        .drone_position
                        .set_x(plan.mission.plannedHomePosition[0]);
                    self.path_info
                        .drone_position
                        .set_y(plan.mission.plannedHomePosition[1]);
                    self.path_info.goal_position = Some(geo::Point::new(
                        plan.mission.items.last().unwrap().params[4].unwrap(),
                        plan.mission.items.last().unwrap().params[5].unwrap(),
                    ));
                }
                // reset planned paths
                self.path_info.optimal_path_from_take_off_to_goal.clear();
                self.path_info.optimal_path_from_goal_to_landing.clear();
            }
            Message::OptionFileNameObstacles(string_value) => {
                self.file_name_obstacles = Some(string_value);
                let file =
                    std::fs::read_to_string(&self.file_name_obstacles.as_ref().unwrap()).unwrap();
                self.plan_obstacles = serde_json::from_str(&file).unwrap();

                // reset planned paths
                self.path_info.optimal_path_from_take_off_to_goal.clear();
                self.path_info.optimal_path_from_goal_to_landing.clear();
            }
            Message::CheckboxObstacles(checkbox_value) => {
                if checkbox_value == true {
                    self.file_name_obstacles = None;
                    self.plan_obstacles = None;
                }
                self.obstacles_from_drone_and_goal_file = checkbox_value;

                // reset planned paths
                self.path_info.optimal_path_from_take_off_to_goal.clear();
                self.path_info.optimal_path_from_goal_to_landing.clear();
            }
            Message::InputDroneLongitudeChanged(string_value) => {
                if let Ok(value) = string_value.trim().replace(',', ".").parse::<f64>() {
                    self.path_info.drone_position.set_x(value);
                }
            }
            Message::InputDroneLatitudeChanged(string_value) => {
                if let Ok(value) = string_value.trim().replace(',', ".").parse::<f64>() {
                    self.path_info.drone_position.set_y(value);
                }
            }
            Message::InputGoalLongitudeChanged(string_value) => {
                if let Ok(value) = string_value.trim().replace(',', ".").parse::<f64>() {
                    if let Some(position) = &mut self.path_info.goal_position {
                        position.set_x(value);
                    } else {
                        self.path_info.goal_position = Some(geo::Point::new(value, 0.0));
                    }
                }
            }
            Message::InputGoalLatitudeChanged(string_value) => {
                if let Ok(value) = string_value.trim().replace(',', ".").parse::<f64>() {
                    if let Some(position) = &mut self.path_info.goal_position {
                        position.set_y(value);
                    } else {
                        self.path_info.goal_position = Some(geo::Point::new(0.0, value));
                    }
                }
            }
            Message::ButtonUpdateWeatherInfo => {
                let url_weather_api = format!("https://api.open-meteo.com/v1/forecast?latitude={:?}&longitude={:?}&hourly=precipitation_probability,precipitation,wind_speed_10m,wind_speed_120m,wind_direction_80m&timezone=Europe%2FBerlin", self.path_info.drone_position.x(), self.path_info.drone_position.y());
                let response = reqwest::blocking::get(url_weather_api);
                match response {
                    Ok(weather_response) => {
                        let weather_info: WeatherData;
                        weather_info = serde_json::from_reader(weather_response).unwrap();
                        if self.weather_info.selected_time.is_none() {
                            self.weather_info.selected_time =
                                weather_info.hourly.time.get(0).map(|t| t.to_string());
                        }
                        self.weather_info.selected_time_index = Some(0);
                        self.weather_info.weather_data = Some(weather_info);
                    }
                    Err(_) => {
                        self.pop_up.text = "Kon geen weersinformatie ophalen. Waarschijnlijk geen internetverbinding".to_string();
                        self.pop_up.show = true;
                    }
                }
            }
            Message::OptionSelectedWeather(time) => {
                self.weather_info.selected_time_index = self
                    .weather_info
                    .weather_data
                    .as_ref()
                    .map(|data| {
                        data.hourly
                            .time
                            .iter()
                            .enumerate()
                            .find_map(
                                |(index, value)| if value == &time { Some(index) } else { None },
                            )
                    })
                    .flatten();
                self.weather_info.selected_time = Some(time);
            }
            Message::PlanRoute => {
                if let Some(weather_data) = self.weather_info.weather_data.as_ref() {
                    let start_heading = f64::from(
                        weather_data.hourly.wind_direction_80m
                            [self.weather_info.selected_time_index.unwrap()],
                    );
                    let end_heading = start_heading;
                    let (start_point, start_heading) =
                        mav_link_plan::get_take_off_waypoint(start_heading);
                    let (end_point, end_heading) = mav_link_plan::get_landing_waypoint(end_heading);

                    let mut astar_planner_builder = AStarPlannerBuilder::new()
                        .start(start_point, start_heading)
                        .goal(self.path_info.goal_position.as_ref().unwrap())
                        .end(end_point, end_heading);
                    if let Some(plan) = self.plan_obstacles.as_ref() {
                        astar_planner_builder.set_geo_fences(&plan.geoFence);
                    } else if let Some(plan) = self.plan_drone_and_goal.as_ref() {
                        astar_planner_builder.set_geo_fences(&plan.geoFence);
                    }
                    let a_star_planner_result = astar_planner_builder.build();

                    match a_star_planner_result {
                        Ok(mut a_star_planner) => {
                            let start = Instant::now();
                            self.path_info.optimal_path_from_take_off_to_goal =
                                a_star_planner.get_optimal_path_to_goal();
                            self.path_info.optimal_path_from_goal_to_landing =
                                a_star_planner.get_optimal_path_from_goal();
                            let duration = start.elapsed();
                            println!("Route geplanned in {:.1?}.", duration);
                        }
                        Err(message) => {
                            self.pop_up.text = message.to_string() + ". Pas deze waarde aan, aub.";
                            self.pop_up.show = true;
                        }
                    }
                }
            }
            Message::SavePressed => {
                if self.weather_info.selected_time.is_none() {
                    self.pop_up.text = "Selecteer een tijd".to_string();
                    self.pop_up.show = true;
                } else if self.path_info.optimal_path_from_take_off_to_goal.len() == 0
                    || self.path_info.optimal_path_from_goal_to_landing.len() == 0
                {
                    self.pop_up.text = "Plan eerst een route".to_string();
                    self.pop_up.show = true;
                } else {
                    let vluchtplan = format!(
                        "vluchtplan_{}.plan",
                        self.weather_info.selected_time.as_ref().unwrap()
                    );
                    self.save_plan_to_file(&vluchtplan);
                }
            }
            Message::PopUpPressed => {
                self.pop_up.show = false;
            }
        }
    }

    fn view(&self) -> Element<Message> {
        let file_name_drone_and_goal_text = text(format!("Drone en doel")).size(24);
        let file_drone_and_goal_picklist = pick_list(
            load_dot_plan_files(),
            self.file_name_drone_and_goal.clone(),
            Message::OptionFileNameDroneAndGoal,
        )
        .placeholder("Kies een bestand...");

        let checkbox_drone_and_goal_as_obstacle = checkbox(
            "Obstakels uit dit bestand halen",
            self.obstacles_from_drone_and_goal_file,
            Message::CheckboxObstacles,
        );

        let file_name_obstacles_text = text(format!("Obstakels")).size(24);
        let file_name_list: Vec<String> = if self.obstacles_from_drone_and_goal_file {
            vec![]
        } else {
            load_dot_plan_files()
        };
        let file_obstacles_picklist = pick_list(
            file_name_list,
            self.file_name_obstacles.clone(),
            Message::OptionFileNameObstacles,
        )
        .placeholder("Kies een bestand...")
        .width(Length::Fill);

        let start_location_text = text(format!("Drone")).size(24);
        let drone_longitude =
            TextInput::new("Longitude", &self.path_info.drone_position.x().to_string())
                .on_input(Message::InputDroneLongitudeChanged)
                .width(Length::Fixed(190.0));
        let drone_latitude =
            TextInput::new("Latitude ", &self.path_info.drone_position.y().to_string())
                .on_input(Message::InputDroneLatitudeChanged)
                .width(Length::Fixed(190.0));

        let goal_longitude = TextInput::new(
            "Longitude",
            &self
                .path_info
                .goal_position
                .unwrap_or_else(|| geo::Point::new(0.0, 0.0))
                .x()
                .to_string(),
        )
        .on_input(Message::InputGoalLongitudeChanged)
        .width(Length::Fixed(190.0));
        let goal_latitude = TextInput::new(
            "Latitude ",
            &self
                .path_info
                .goal_position
                .unwrap_or_else(|| geo::Point::new(0.0, 0.0))
                .y()
                .to_string(),
        )
        .on_input(Message::InputGoalLatitudeChanged)
        .width(Length::Fixed(190.0));

        let left_column = column![
            vertical_space(20),
            file_name_drone_and_goal_text,
            file_drone_and_goal_picklist,
            checkbox_drone_and_goal_as_obstacle,
            vertical_space(5),
            file_name_obstacles_text,
            file_obstacles_picklist,
            vertical_space(20),
            start_location_text,
            row![
                column!["Longitude:", vertical_space(10), "Latitude:"].align_items(Alignment::End),
                column![drone_longitude, drone_latitude]
            ]
            .align_items(Alignment::Center),
            vertical_space(30),
            text(format!("Doel")).size(24),
            row![
                column!["Longitude:", vertical_space(10), "Latitude: "].align_items(Alignment::End),
                column![goal_longitude, goal_latitude]
            ]
            .align_items(Alignment::Center),
            vertical_space(30),
        ]
        .width(Length::Fill)
        .align_items(Alignment::Center)
        .spacing(10);

        let text_wind_direction_80m = self
            .weather_info
            .weather_data
            .as_ref()
            .map(|data| {
                let direction = data
                    .hourly
                    .wind_direction_80m
                    .get(self.weather_info.selected_time_index.unwrap_or_default())
                    .map_or(String::new(), |d| d.to_string());

                let unit = data.hourly_units.wind_direction_80m.as_str();

                text(format!("{}{}", direction, unit)).size(18)
            })
            .unwrap_or_else(|| text("---").size(18));

        let text_wind_speed_10m = self
            .weather_info
            .weather_data
            .as_ref()
            .map(|data| {
                let speed = data
                    .hourly
                    .wind_speed_10m
                    .get(self.weather_info.selected_time_index.unwrap_or_default())
                    .map_or(0.0, |d| *d);
                text(format!("{} Bft", kmh_to_beaufort(speed))).size(18)
            })
            .unwrap_or_else(|| text("---").size(18));

        let text_wind_speed_120m = self
            .weather_info
            .weather_data
            .as_ref()
            .map(|data| {
                let speed = data
                    .hourly
                    .wind_speed_120m
                    .get(self.weather_info.selected_time_index.unwrap_or_default())
                    .map_or(0.0, |d| *d);
                text(format!("{} Bft", kmh_to_beaufort(speed))).size(18)
            })
            .unwrap_or_else(|| text("---").size(18));

        let text_precipitation = self
            .weather_info
            .weather_data
            .as_ref()
            .map(|data| {
                let precipitation = data
                    .hourly
                    .precipitation
                    .get(self.weather_info.selected_time_index.unwrap_or_default())
                    .map_or(String::new(), |d| d.to_string());

                let unit = data.hourly_units.precipitation.as_str();

                text(format!("{} {}", precipitation, unit)).size(18)
            })
            .unwrap_or_else(|| text("---").size(18));

        let text_precipitation_probability = self
            .weather_info
            .weather_data
            .as_ref()
            .map(|data| {
                let precipitation_probability = data
                    .hourly
                    .precipitation_probability
                    .get(self.weather_info.selected_time_index.unwrap_or_default())
                    .map_or(String::new(), |d| d.to_string());

                let unit = data.hourly_units.precipitation_probability.as_str();

                text(format!("{}{}", precipitation_probability, unit)).size(18)
            })
            .unwrap_or_else(|| text("---").size(18));

        let button_weather = Button::new("Update").on_press(Message::ButtonUpdateWeatherInfo);

        let times = self
            .weather_info
            .weather_data
            .as_ref()
            .map(|checkbox_drone_and_goal_as_obstacle| {
                checkbox_drone_and_goal_as_obstacle.hourly.time.clone()
            })
            .unwrap_or(Vec::new());
        let picklist_time = pick_list(
            times,
            self.weather_info.selected_time.clone(),
            Message::OptionSelectedWeather,
        )
        .placeholder("Kies een tijd...");

        let middle_column = column![
            vertical_space(20),
            text(format!("Weersinformatie")).size(24),
            row![
                column![
                    text(format!("Windrichting (80 m):")).size(18),
                    vertical_space(5),
                    text(format!("Windkracht   (10 m):")).size(18),
                    vertical_space(5),
                    text(format!("Windkracht (120 m):")).size(18),
                    vertical_space(5),
                    text(format!("Neerslaghoeveelheid:")).size(18),
                    vertical_space(5),
                    text(format!("Neerslagkans:")).size(18),
                ]
                .align_items(Alignment::End),
                column![
                    text_wind_direction_80m,
                    vertical_space(5),
                    text_wind_speed_10m,
                    vertical_space(5),
                    text_wind_speed_120m,
                    vertical_space(5),
                    text_precipitation,
                    vertical_space(5),
                    text_precipitation_probability
                ]
                .align_items(Alignment::End)
                .width(Length::Fixed(80.0))
            ],
            button_weather,
            vertical_space(20),
            text(format!("Wanneer wil je vliegen?")).size(24),
            picklist_time,
            vertical_space(20),
        ]
        .width(Length::Fixed(400.0))
        .align_items(Alignment::Center)
        .spacing(10);

        let button_astar_enabled: Option<Message>;
        if self.weather_info.selected_time.is_some()
            && self.path_info.goal_position.is_some()
            && (!self.obstacles_from_drone_and_goal_file && self.plan_obstacles.is_some())
            || (self.obstacles_from_drone_and_goal_file && self.plan_obstacles.is_none())
        {
            button_astar_enabled = Some(Message::PlanRoute);
        } else {
            button_astar_enabled = None;
        }
        let button_astar = Button::new("Plan Route").on_press_maybe(button_astar_enabled);
        let button_save_enabled: Option<Message>;
        if self.weather_info.selected_time.is_some()
            && !self.path_info.optimal_path_from_goal_to_landing.is_empty()
            && !self.path_info.optimal_path_from_take_off_to_goal.is_empty()
        {
            button_save_enabled = Some(Message::SavePressed);
        } else {
            button_save_enabled = None;
        }
        let button_save = Button::new("Opslaan").on_press_maybe(button_save_enabled);

        let right_column = column![
            vertical_space(20),
            text(format!("Vluchtplan")).size(24),
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
            .width(Length::Fill)
            .height(Length::Fill)
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
    fn save_plan_to_file(&mut self, file_name_drone_and_goal: &String) {
        let mut plan: MavLinkPlan = MavLinkPlan::new()
            .add_take_off_sequence(f64::from(
                self.weather_info
                    .weather_data
                    .as_ref()
                    .unwrap()
                    .hourly
                    .wind_direction_80m[self.weather_info.selected_time_index.unwrap()],
            ))
            .add_path(&self.path_info.optimal_path_from_take_off_to_goal[1..])
            .add_goal_position(
                &self
                    .path_info
                    .goal_position
                    .unwrap_or_else(|| geo::Point::new(0.0, 0.0)),
            )
            .add_path(
                &self.path_info.optimal_path_from_goal_to_landing
                    [..&self.path_info.optimal_path_from_goal_to_landing.len() - 1],
            )
            .add_landing_sequence(f64::from(
                self.weather_info
                    .weather_data
                    .as_ref()
                    .unwrap()
                    .hourly
                    .wind_direction_80m[self.weather_info.selected_time_index.unwrap()],
            ));

        if let Some(source_plan) = self.plan_drone_and_goal.as_ref() {
            plan.copy_geo_fences(&source_plan);
        }

        // Create a file to save the formatted JSON
        let file = File::create(&file_name_drone_and_goal).expect("Failed to create file");
        let writer = BufWriter::new(file);

        // Serialize and format the data with newlines and indentation
        serde_json::to_writer_pretty(writer, &plan).expect("Failed to write JSON data to file");

        Notification::new()
            .summary("Bestand Opgeslagen")
            .body(&file_name_drone_and_goal)
            .show()
            .unwrap();
    }
}

fn load_dot_plan_files() -> Vec<String> {
    let mut file_list = Vec::new();
    // Get the current directory's contents.
    if let Ok(entries) = std::fs::read_dir(".") {
        for entry in entries {
            if let Ok(entry) = entry {
                let path = entry.path();
                // Check if it's a file and has a `.plan` extension.
                if path.is_file() && path.extension() == Some(std::ffi::OsStr::new("plan")) {
                    // Print the file name.
                    if let Some(filename) = path.file_name() {
                        if let Some(filename_str) = filename.to_str() {
                            file_list.push(filename_str.to_string());
                        }
                    }
                }
            }
        }
    }

    file_list.sort();
    return file_list;
}

/// Converts wind speed from kilometers per hour (km/h) to the Beaufort scale.
///
/// This function maps a wind speed in km/h to the corresponding Beaufort number,
/// ranging from 0 (calm) to 12 (hurricane force).
///
/// # Arguments
///
/// * `speed` - A floating point number representing the wind speed in kilometers per hour.
///
/// # Returns
///
/// An integer representing the wind speed on the Beaufort scale.
///
/// # Examples
///
/// ```
/// let beaufort_scale = kmh_to_beaufort(55.0);
/// assert_eq!(beaufort_scale, 7); // Near gale
/// ```
fn kmh_to_beaufort(speed: f64) -> i32 {
    match speed {
        speed if speed < 1.0 => 0,
        speed if speed < 6.0 => 1,
        speed if speed < 12.0 => 2,
        speed if speed < 20.0 => 3,
        speed if speed < 29.0 => 4,
        speed if speed < 39.0 => 5,
        speed if speed < 50.0 => 6,
        speed if speed < 62.0 => 7,
        speed if speed < 75.0 => 8,
        speed if speed < 89.0 => 9,
        speed if speed < 103.0 => 10,
        speed if speed < 118.0 => 11,
        _ => 12,
    }
}

pub fn main() -> iced::Result {
    MavlinkPlanGenerator::run(Settings::default())
}
