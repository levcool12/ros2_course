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

## Defect note (stage 2)

Reproduced LIVE with config/bridge-broken.yaml (inverted IMU direction + misspelled
cmd_vel2 gz topic): /vehicle/imu visible in ros2 topic list but silent, gz-side IMU publisher
healthy, bridge published commands into the void (no subscriber on cmd_vel2). Fixed by relaunching
with config/bridge.yaml - IMU flowed again. Full transcript: evidence/pr07/bridge-defect.txt.
