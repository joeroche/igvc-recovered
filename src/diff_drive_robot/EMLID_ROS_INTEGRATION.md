# Emlid RS+ and ROS2 Integration

## What is Emlid RS+?

The Emlid Reach RS+ is a high-precision GNSS receiver designed for RTK positioning. It receives satellite signals from GPS, GLONASS, Galileo, and BeiDou, then combines those measurements with correction data from an RTK base station or network source. This allows centimeter-level position accuracy instead of the meter-level accuracy of standalone GPS.

## How Emlid works

- The RS+ collects raw satellite observation data (pseudorange and carrier phase).
- It computes a precise position in geographic coordinates (latitude, longitude, altitude).
- When connected to a base station or network correction stream, it enters RTK mode and refines the position:
  - RTK fixes use integer ambiguity resolution.
  - The resulting position can be accurate to a few centimeters.
- The RS+ can output NMEA sentences over TCP or UDP, or serve data via its REST and Web UI interface.

## How this ROS2 integration is structured

### `emlid_gps_node`

This node connects to the Emlid RS+ using a configurable network endpoint.
It reads NMEA sentences from the receiver and publishes:

- `/fix` as `sensor_msgs/NavSatFix`
- `/fix_status` as `sensor_msgs/NavSatStatus`
- `/emlid/pose` as `geometry_msgs/PoseStamped` in a local ENU frame

The local pose is computed using the first configured origin (`origin_lat`, `origin_lon`, `origin_alt`).
This allows the GNSS solution to be expressed in a local map-relevant coordinate frame.

### `waypoint_driver`

This node drives the robot toward a list of coordinate points.
It subscribes to `/odometry/filtered` from the localizer and publishes velocity commands to `/waypoint_cmd_vel`.

The waypoint driver:

- reads goals from `config/waypoints.yaml`
- computes the vector between the robot pose and the next goal
- turns toward the goal and moves forward
- stops when the goal is reached, then advances to the next point

### `behavior_node`

The existing behavior node now subscribes to `/waypoint_cmd_vel`.
This means waypoint navigation is integrated into the same collision-avoidance and camera/lidar arbitration stack.

Priority order in behavior arbitration is now:

1. critical lidar obstacle avoidance
2. camera-based obstacle avoidance
3. waypoint navigation commands
4. SLAM map avoidance
5. line following

## How GNSS and local mapping work together

1. `emlid_gps_node` publishes high-accuracy GPS fixes.
2. The robot localization stack can fuse these GPS fixes with IMU, wheel odometry, and other sensors.
3. The localized output (`/odometry/filtered`) gives the robot a stable pose in the map/odom frame.
4. `waypoint_driver` uses that pose to navigate to fixed coordinates on the localized map.

## How to use it

1. Start your simulation or robot stack:

```bash
ros2 launch diff_drive_robot autonomous_drive.launch.py
```

2. The launch will start:
- `emlid_gps_node`
- `waypoint_driver`
- SLAM/localization
- the behavior stack
- two RViz windows (full view + 2D path)

3. Adjust `config/waypoints.yaml` with the coordinates you want the robot to visit.

## Notes

- Coordinates in `waypoint_driver` are expected in the same frame as `/odometry/filtered`.
- The Emlid node publishes a local ENU pose for easier map alignment.
- If you run with a real RS+ receiver, point the node at the device IP and port.
- For full map-based navigation, fuse GNSS with robot localization using `robot_localization` and `navsat_transform_node`.
