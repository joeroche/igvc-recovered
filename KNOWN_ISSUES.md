# Current major issues

- **Simulation can touch hardware.** The main launch path starts Gazebo alongside physical ODrive and GPS nodes. These need separate launch files before use.
- **Motion does not reliably fail safe.** Commands are cached without freshness checks, and losing the line can still produce forward motion.
- **Physical bringup is incomplete.** The main package does not launch a complete camera, lidar, IMU, odometry, and TF stack.
- **Motor configuration is uncertain.** Wheel geometry differs across files, and ODrive startup may recalibrate or report success after a state failure.
- **The Gazebo bridge config is invalid YAML.** This can prevent simulated sensor topics from starting.
- **Perception is only partly connected.** The active path is RGB line following; the obstacle-command input has no working producer, while useful OAK-D and depth-grid experiments remain under `prototypes/`.

See [`ARCHITECTURE.md`](ARCHITECTURE.md) for the system flow.
