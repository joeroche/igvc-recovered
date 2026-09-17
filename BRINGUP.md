# Bringup plan

Use this order when moving from simulation to the robot.

1. Build on Ubuntu 22.04 with ROS 2 Humble. Repair package dependencies and the invalid bridge YAML first.
2. Separate `sim.launch.py` and `hardware.launch.py`. Physical motor initialization must require an explicit opt-in and must never be included by the default simulation launch.
3. Exercise simulation without ODrive or Emlid. Verify camera, scan, IMU, odometry, TF, SLAM, line command, arbitration, and Gazebo command topics.
4. Add automated liveness tests: stop each input publisher and verify the arbiter emits zero within a bounded timeout.
5. Validate the hardware E-stop independently of ROS. Raise the drive wheels, limit current, and make ODrive calibration a separate explicit procedure.
6. Confirm actual wheel radius/separation, motor directions, encoder CPR, gear ratio, serial/device aliases, and controller configuration.
7. Bring up physical sensors one at a time. Record topics, rates, frame IDs, timestamps, and disconnect behavior for lidar, camera, IMU, and GPS.
8. Establish encoder-derived odometry and one authoritative odometry topic. Then validate EKF covariances and the `odom -> base_link` transform.
9. Run bag-replay tests for line following and obstacle handling before low-speed field tests.
10. Test integrated motion at low speed with an operator on the hardware E-stop.
