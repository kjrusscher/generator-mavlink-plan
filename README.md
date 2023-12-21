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

SkyOps

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