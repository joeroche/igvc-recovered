# Rutgers IGVC Software

This repository brings the Rutgers IGVC software, electrical work, older robot code, and perception experiments into one place for the team.

The main ROS 2 package is [`src/diff_drive_robot`](src/diff_drive_robot). It includes line following, command arbitration, lidar and GPS code, waypoint driving, ODrive control, mapping configuration, and Gazebo simulation.

> **Do not run the current top-level launch file on connected motors.** It currently mixes simulation with physical ODrive and GPS nodes.

## Start here

- [`PROJECT_MAP.md`](PROJECT_MAP.md): what is current, old, or experimental.
- [`KNOWN_ISSUES.md`](KNOWN_ISSUES.md): the few major blockers visible right now.
- [`BRINGUP.md`](BRINGUP.md): the path from simulation to robot testing.

## Future improvements

The biggest opportunity is perception. The current path mainly uses an OpenCV RGB line detector, while the OAK-D and point-cloud work under [`prototypes`](prototypes) is not connected to the main stack.

The next version can combine RGB and depth for stronger line detection and obstacle awareness, then add a lightweight segmentation or obstacle model that runs on the robot's RTX 3060. That would turn the existing perception experiments into a clear upgrade path instead of leaving them as separate prototypes.

More detail is available in [`ARCHITECTURE.md`](ARCHITECTURE.md) and [`HARDWARE.md`](HARDWARE.md).
