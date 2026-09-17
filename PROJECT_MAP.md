# Project map

| Path | Role |
|---|---|
| [`src/diff_drive_robot`](src/diff_drive_robot) | Main ROS 2 package and starting point |
| [`hardware/rutgers_igvc_electrical`](hardware/rutgers_igvc_electrical) | Electrical, E-stop, and motor-control work |
| [`hardware/rutgers_igvc_electrical_p3_2026`](hardware/rutgers_igvc_electrical_p3_2026) | Newer electrical copy with a different ODrive config |
| [`legacy/ros1_p5`](legacy/ros1_p5) | Older ROS 1 autonomy, sensors, setup, and teleop |
| [`legacy/igvc21_22`](legacy/igvc21_22) | 2021–22 robot code and electronics |
| [`prototypes`](prototypes) | Disconnected OAK-D, depth-grid, and small ROS 2 experiments |

Use `src/diff_drive_robot` for new work. Use `legacy/` for older hardware knowledge and `prototypes/` for perception ideas worth bringing into the main package. Keep the two electrical copies separate until they can be compared with the live robot.
