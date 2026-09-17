# Hardware

## Compute

- Intel NUC12DCMi9 host.
- Ubuntu 22.04.5 LTS with kernel 6.8.0-136-generic.
- NVIDIA GeForce RTX 3060.

## Main devices

| Device or subsystem | What is here | Integration status |
|---|---|---|
| ODrive motor controller | ROS 2 Python node, JSON configuration, legacy/manual scripts | Active node exists; initialization and geometry issues remain |
| Emlid Reach-class GNSS | ROS 2 TCP client and integration notes | Launched, but endpoint/origin values require team validation |
| RPLIDAR | Python driver/test and `/scan` consumers | Driver not registered or launched in current package |
| RGB/depth cameras | Gazebo camera models; RealSense and DepthAI workspaces/configuration | Simulation streams are active; physical camera launch not found |
| Emergency stop | ESP32 transmitter/receiver firmware and PCB files | Hardware validation still needed |
| Power distribution | Draw.io diagrams and rendered images | Reference material is available |

Device aliases, serial paths, radio addresses, GPS endpoints, and calibration constants still need to be checked on the physical robot.

There are two different ODrive configurations in the hardware folders. Compare both against a live controller readback before writing either one to the robot.

## Safety boundary

The ODrive watchdog stops after 0.5 seconds without a controller message, but the behavior node can keep sending an old command. This is not a substitute for the hardware E-stop.

## Electrical files

`hardware/rutgers_igvc_electrical` contains:

- E-stop transmitter and receiver PlatformIO projects.
- E-stop schematic/layout PDFs and editable project files.
- Power-distribution and protection diagrams.
- ODrive configuration JSON.
- Manual motor-control and controller-input scripts.
