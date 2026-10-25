# contracts/topics.md — PR07 topic contract

Reconstructed from the PR07 assignment (the official course kit was not
available). All topics are absolute; node namespaces do not rename them.

## Sensors and state (Gazebo -> ROS via ros_gz_bridge)

| ROS topic                 | ROS 2 type               | Gazebo topic                                  | frame_id                | rate (SDF) | QoS                  | time source |
|--------------------------|--------------------------|-----------------------------------------------|-------------------------|------------|----------------------|-------------|
| /vehicle/camera/image_raw| sensor_msgs/msg/Image    | /model/course_ackermann/camera/image          | camera_link             | 25 Hz *    | sensor data (BE, KL) | simulation  |
| /vehicle/scan            | sensor_msgs/msg/LaserScan| /model/course_ackermann/scan                  | lidar_link              | 10 Hz      | sensor data (BE, KL) | simulation  |
| /vehicle/imu             | sensor_msgs/msg/Imu      | /model/course_ackermann/imu                   | imu_link                | 100 Hz     | sensor data (BE, KL) | simulation  |
| /vehicle/odom_gz         | nav_msgs/msg/Odometry    | /model/course_ackermann/odometry              | course_ackermann/odom   | 50 Hz      | reliable             | simulation  |
| /vehicle/odom (adapter)  | nav_msgs/msg/Odometry    | —                                             | odom                    | 50 Hz      | reliable             | simulation  |
| /clock                   | rosgraph_msgs/msg/Clock  | /clock                                        | —                       | 1000 Hz    | —                    | simulation  |

\* camera update_rate is the one sensor parameter changed for PR07
(kit default 20 Hz -> 25 Hz).

## Commands (ROS -> Gazebo)

| ROS topic          | ROS 2 type                   | Gazebo topic                            | direction | QoS      |
|--------------------|------------------------------|-----------------------------------------|-----------|----------|
| /drive/safe        | ackermann_msgs/msg/AckermannDriveStamped | — (adapter input)          | in        | reliable |
| /vehicle/gz_cmd_vel| geometry_msgs/msg/Twist      | /model/course_ackermann/cmd_vel         | ROS_TO_GZ | reliable |

/drive/safe never reaches Gazebo directly: only the adapter output
/vehicle/gz_cmd_vel crosses the bridge. /drive/raw must not be bridged.

## Drive adapter contract

| Input                       | Output                                              |
|-----------------------------|-----------------------------------------------------|
| drive.speed (finite)        | v = clamp(speed, 0.0, 1.2); linear.x = v m/s        |
| drive.steering_angle (finite)| d = clamp(angle, -0.45, 0.45); angular.z = v*tan(d)/0.8 rad/s |
| NaN/Inf in any field, or header.stamp age > 0.25 s | zero Twist + diagnostic reason; previous command not repeated |

## Odometry adapter contract

- Input frames must be exactly `course_ackermann/odom` ->
  `course_ackermann/base_link`; anything else, or non-finite values, is
  rejected.
- Output renames frames to `odom` -> `base_link`, preserving stamp,
  pose, twist and both covariance matrices.
- The adapter publishes the single dynamic tf `odom -> base_link`;
  static frames belong to robot_state_publisher (PR06).
