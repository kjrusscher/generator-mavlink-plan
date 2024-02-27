# How to use
## For Ubuntu 20.04
```
docker-compose up build-for-20.04
```
This compiles the application and puts the following files and directory in `flight-route-planner-20`:
- flight-route-planner (executable)
- install.sh 
- uninstall.sh
- flight-route-planner.png (icon for .desktop file)
- plan-bestanden (directory for .plan files)

`./install.sh` creates `flight-route-planner.desktop` with links to the `flight-route-planner-20` folder.
These are the manual steps to make it work on Ubuntu 20.04 (this is documentation, it's now part of the install script):
- Set permission of `./install.sh` to executable, either via the file browser or with `chmod +x`.
- Go to `preferences->behaviour->executable text files` in the file browser and select `ask each time`.
- Run `./install.sh` by double clicking it in the file browser.
- Create shortcut for `~/.local/share/application/flight-plan-generator.desktop` on Desktop.
- Right click shortcut and select `Allow launch`.
- Application can now be found in launcher.

## For Ubuntu 22.04
```
docker-compose up build-for-22.04
```
This results in executable plan-generator-22-04. 
Running the `./install.sh` file will make the desktop application visible in launcher.

# Other documentation
## Avy Parameters
| Parameter | Value |
|-----------------|----------:|
| Maximum yaw rate | 45 deg/s |
| Maximum rate of climb | 4 m/s |
| Maximum rate of descent | 5 m/s |
| Climb angle | nominal 8 deg / 14% |
| | max headwind 15 deg / 26% |
| | mad tailwind 5 deg / 10% |
| Descent angle | nominal 10 deg / 18% |
| | max headwind 18 deg / 33% |
| | mad tailwind 7 deg / 12% |
| Turn rate limit | 13 deg/s |
| Stall airspeed | 22 m/s (80 km/h) |
| Minimum airspeed | 25 m/s (90 km/h) |
| Nominal cruise airspeed | 28 m/s (101 km/h) |
| Max cruise airspeed | 32 m/s (115 km/h) |
| Never exceed airspeed | 35 m/s (126 km/h) |

## A*

Check if start and end position are valid.

Push start position to Open Set

while (not end)
    Pull top position from Open Set
    calculate new positions

    Check if new position are in Closed Set
    Check validity of new positions
    Check if new positions is end 

    Calculate node information of new position

    Push top position to Closed Set
    Push new positions to Open Set


functions
- calculate_new_positions(geo::Point, azimuth) -> Vec<geo::Point>
- is_position_valid(geo::Point) -> bool
- calculate_h(geo::Point) -> f64 {dubins_path}
- calculate_g(geo::Point, top_position: geo::Point) -> f64

Vec<geo::Point> can also be array as there are always the same amount of points calculated.
Does geo::Point include heading/azimuth?

Cost so far (g):
- Path length
- Steering difference
- Total amount of steering

Cost to goal (h):
- Calculate heading change begin and end
- Calculate circle begin and end
- Calculate distance between new points