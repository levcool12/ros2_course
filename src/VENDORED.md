# Vendored build dependencies

Upstream sources pinned inside the workspace because the local WSL
environment has no sudo access to apt, and CI has no rosdep install of
these packages. Remove them once the system packages are available.

- `xacro/` — https://github.com/ros/xacro, tag 2.0.12 (ROS 2 Jazzy
  release). Pure python, used to expand `course_description/urdf/*.xacro`.
- `ackermann_msgs/` — https://github.com/ros-drivers/ackermann_msgs,
  branch ros2 (commit 81a5f2d). Message definitions for PR07.
