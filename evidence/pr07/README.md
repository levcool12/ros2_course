# PR07 evidence status

## What is complete

- src/course_simulation: drive adapter, odom adapter, bridge config,
  reconstructed world + model, simulation.launch.py, 20 unit tests.
- tests.txt: 363 tests, 0 failures (adapters, bridge contract, workspace).
- odom-adapter.md: frame normalization, tf ownership, stamp-preservation test.
- sensors.json: declared topics/types/frames/rates/QoS (measured = null).
- Stage-2 defect class reproduced structurally: config/bridge-broken.yaml
  (inverted IMU direction, misspelled cmd_vel gz topic) is detected by
  test_bridge_config.py::test_broken_copy_is_detected.

## What is blocked and why

This environment (WSL2 Ubuntu 24.04, ROS 2 Jazzy) has NO sudo access and
NO Gazebo installed; the official course kit (.course-kit with the real
models/worlds/bridge.yaml and check_practice.py) is absent. Therefore the
live parts could not be executed honestly:
  - headless.txt (live headless launch log),
  - bag-info.txt + fixtures/pr07-sensors (>=30 s sim-time recording),
  - measured frequencies in sensors.json,
  - live defect diagnosis via ros2 topic info -v / gz topic -i.

## How to finish (once deps + kit are available)

    sudo apt update && sudo apt upgrade
    sudo apt install ros-jazzy-ackermann-msgs ros-jazzy-ros-gz
    # (vendored src/ackermann_msgs and src/xacro can then be removed)
    # unpack the course kit into .course-kit/ and copy its models/worlds/
    # config into src/course_simulation if they differ from the reconstruction

    cd ~/ros2_course
    colcon build --symlink-install --packages-up-to course_simulation
    source install/setup.bash
    mkdir -p evidence/pr07 fixtures
    export GZ_SIM_RESOURCE_PATH="$PWD/src/course_simulation/models${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
    ros2 launch course_simulation simulation.launch.py world:=$PWD/src/course_simulation/worlds/mcity-minimal.sdf bridge_config:=$PWD/src/course_simulation/config/bridge.yaml headless:=true 2>&1 | tee evidence/pr07/headless.txt

    # terminal 2: verify the full path /drive/safe -> adapter -> bridge -> gz -> odom
    ros2 topic info -v /vehicle/gz_cmd_vel
    gz topic -i -t /model/course_ackermann/cmd_vel
    ros2 topic echo /clock --once
    ros2 topic hz /vehicle/scan
    ros2 topic pub -r 10 /drive/safe ackermann_msgs/msg/AckermannDriveStamped "{header: auto, drive: {speed: 0.5, steering_angle: 0.2}}"
    ros2 topic echo /vehicle/gz_cmd_vel --once
    timeout 3s gz topic -e -t /model/course_ackermann/cmd_vel
    ros2 topic echo /vehicle/odom --once
    ros2 run tf2_ros tf2_echo odom base_link --ros-args -p use_sim_time:=true

    # terminal 3: >=30 s of simulation time
    timeout --preserve-status --signal=INT --kill-after=5s 35s \
      ros2 bag record --storage mcap --use-sim-time -o fixtures/pr07-sensors \
      --topics /vehicle/camera/image_raw /vehicle/scan /vehicle/imu /vehicle/odom /tf /tf_static /clock
    ros2 bag info fixtures/pr07-sensors | tee evidence/pr07/bag-info.txt

    # measure real rates (ros2 topic hz) and fill measured_rate_hz in sensors.json,
    # run the live defect demo with config/bridge-broken.yaml:
    #   copy it over config (or pass bridge_config:=.../bridge-broken.yaml),
    #   watch /vehicle/imu stay silent while gz side still publishes,
    #   then restore bridge.yaml and re-verify.

    python3 .course-kit/v1/tools/check_practice.py PR07 --submission .

After that, update report.json (headless_verified, recording_verified ->
true) and commit the new evidence.
