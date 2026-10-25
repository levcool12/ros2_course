# PR07 odom adapter

## Frames: source and normalized

| field            | source (Gazebo AckermannSteering) | normalized (this adapter) |
|------------------|-----------------------------------|---------------------------|
| header.frame_id  | course_ackermann/odom             | odom                      |
| child_frame_id   | course_ackermann/base_link        | base_link                 |

Everything else is copied unchanged: header.stamp, pose (position +
orientation), twist (linear + angular) and both covariance matrices
(pose.covariance, twist.covariance).

Messages with any other source frames, or with non-finite values, are
rejected and logged - never republished (contracts/topics.md).

## tf ownership

| transform        | owner                                     | channel |
|------------------|-------------------------------------------|---------|
| odom -> base_link| odom_adapter (dynamic, from each message) | /tf     |
| base_link -> camera_link / lidar_link / imu_link | robot_state_publisher (PR06) | /tf_static |
| wheel joints     | not published in this PR (no joint_states source) | — |

Exactly one node publishes the dynamic odom -> base_link edge: the
adapter subscribes /vehicle/odom_gz (bridge output) and is the only
publisher of both /vehicle/odom and this tf edge. The bridge itself is
not remapped to /tf, so it cannot compete for the edge.

## Timestamp preservation test

test_odom_adapter.py::test_stamp_pose_twist_covariances_preserved feeds a
message with stamp sec=42, nanosec=500000000 and asserts the normalized
output carries exactly the same stamp, plus position (1.5, -0.25, 0),
twist (0.8 linear.x, 0.1 angular.z) and both covariances.

Verified with: colcon test --packages-select course_simulation
(363 tests, 0 failures across the workspace; see tests.txt).

## Defect note (stage 2, structural reproduction)

The live reproduction (wrong IMU direction / misspelled cmd_vel topic in
a bridge copy, diagnosed via ros2 topic info -v and gz topic -i) requires
a running Gazebo, which is not installed in this environment (no sudo
access). The defect class is reproduced structurally instead:
config/bridge-broken.yaml contains both errors, and
test_bridge_config.py::test_broken_copy_is_detected proves the validator
catches them (IMU direction ROS_TO_GZ, cmd_vel routed to
/model/course_ackermann/cmd_vel2). See evidence/pr07/README.md for the
exact live procedure to finish once dependencies are installed.
