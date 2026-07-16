# Go2 description

The URDF and DAE assets are sourced from the official
`unitreerobotics/unitree_ros` repository at commit
`d96d8f63ae17a7108d4f7229c00ef875ba7129c9` and retain the included
BSD-3-Clause license.

`urdf/go2_robonix.urdf` adds a zero-offset `base_link -> base` transform and a
zero-offset `radar -> utlidar_lidar` topic-frame alias around the otherwise
unchanged official model. The model already contains the official `imu`,
`radar`, and `front_camera` frames. Their
extrinsics still require verification against the physical hardware/revision
before supervised navigation.

This directory is also the deployment's Robonix robot-description primitive.
It accepts only the URDF served by Soma when it exactly matches the pinned
file, then runs the standard ROS 2 `robot_state_publisher` under
`rmw_cyclonedds_cpp`. It exposes no movement or posture capability.
