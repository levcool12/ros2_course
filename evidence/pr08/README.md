# PR08 evidence status

## Complete

- src/localization_case: launch (map_server + amcl + lifecycle manager,
  one command, sim time), reconstructed AMCL profile + defect copy
  (base_frame_id: chassis), measurement node (sim-time window
  [t0+10, t0+30), 10 Hz lookup map->odom, /amcl_pose covariance median,
  JSON result with measured values), 11 unit tests.
- PR07 recording re-recorded for the PR08 contract: gz_frame_id gives
  header.frame_id = lidar_link (was course_ackermann/lidar_link/lidar);
  37.1 s sim time, all five required topics present (bag-info in git
  history of PR07; fixtures/ kept locally).

## Blocked (two missing inputs, owner: user)

1. nav2 packages: sudo apt install ros-jazzy-nav2-amcl
   ros-jazzy-nav2-map-server ros-jazzy-nav2-lifecycle-manager
   (tf2_ros_py/tf2_tools already present).
2. Weekly course kit unpacked to .course-kit/ (map.yaml/map.pgm,
   kit amcl.yaml profile, contract.json, localization schema,
   validate_localization.py, check_practice.py). Until then the kit
   AMCL profile must replace the reconstructed config/amcl.yaml.

## Procedure after unblocking (terminals per assignment)

  T1: ros2 launch localization_case localization.launch.py \
        params_file:=$PWD/src/localization_case/config/amcl.yaml \
        map:=$PWD/.course-kit/v1/fixtures/pr08/map.yaml
      ros2 lifecycle get /map_server ; ros2 lifecycle get /amcl   # active
  T2: ros2 run localization_case measure_localization.py --ros-args \
        -p use_sim_time:=true -- --map-yaml .course-kit/v1/fixtures/pr08/map.yaml \
        --map-pgm .course-kit/v1/fixtures/pr08/map.pgm \
        --bag-path $PWD/fixtures/pr07-sensors \
        --params-file $PWD/src/localization_case/config/amcl.yaml \
        --out $PWD/evidence/pr08/localization.json
      # wait for "waiting for the first /clock"
  T3: ros2 bag play fixtures/pr07-sensors     # no --clock, no --loop
  # defect: same with amcl-defect.yaml, a few seconds, stop launch first
  # then: validate_localization.py -> schema-check.txt, fill tf.md,
  # cp map.* evidence/pr08/, report.json defect flags -> true, commit B2.
