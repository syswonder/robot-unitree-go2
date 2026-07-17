# Go2 description

The URDF and DAE assets are sourced from the official
`unitreerobotics/unitree_ros` repository at commit
`d96d8f63ae17a7108d4f7229c00ef875ba7129c9` and retain the included
BSD-3-Clause license.

`urdf/go2_robonix.urdf` adds a zero-offset `base_link -> base` transform and a
zero-offset `radar -> utlidar_lidar` topic-frame alias around the otherwise
unchanged official model. It also connects the live MID-360 IMU topic frame as
`utlidar_lidar -> utlidar_imu` using Livox's published chip position
`(0.011, 0.02329, -0.04412) m` and aligned axes from the
[MID-360 User Manual v1.2](https://terra-1-g.djicdn.com/851d20f7b9f64838a34cd02351370894/Livox/Livox_Mid-360_User_Manual_CHS.pdf).
The model already contains the official Go2 `imu`, `radar`, and `front_camera`
frames. The robot-mounted radar and camera extrinsics still require physical
measurement/verification before supervised navigation.

This directory is also the deployment's Robonix robot-description primitive.
It accepts only the URDF served by Soma when it exactly matches the pinned
file, then runs the standard ROS 2 `robot_state_publisher` under
`rmw_cyclonedds_cpp`. It exposes no movement or posture capability.
