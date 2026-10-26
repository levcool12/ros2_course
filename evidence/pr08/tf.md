# PR08 tf contract and defect transcript (template + live outputs)

## Owners of the three transforms (single publisher each)

| transform        | owner (during replay)                        | channel    |
|------------------|----------------------------------------------|------------|
| map -> odom      | AMCL (nav2_amcl) while localizing            | /tf, dynamic |
| odom -> base_link| PR07 odometry adapter, REPLAYED FROM THE BAG (no live publisher) | /tf, dynamic |
| base_link -> lidar_link | PR07 robot_state_publisher, REPLAYED FROM THE BAG (/tf_static, transient local) | /tf_static, static |

During replay the recorded transforms are used; second owners (a live
odometry adapter, a second robot_state_publisher) must NOT be running.
AMCL receives /vehicle/scan, /map and the initial pose (0, 0, 0) from
the profile; it never republishes the two recorded edges.

## The check command (both experiences use the same one)

    ros2 run tf2_ros tf2_echo map odom --ros-args -p use_sim_time:=true

## Defect run (amcl-defect.yaml: base_frame_id: chassis)

STATUS: pending live run - nav2_amcl/nav2_map_server/nav2_lifecycle_manager
are not installed in this environment yet, and the course kit (map,
kit AMCL profile) is not available. To reproduce after installation:

    ros2 launch localization_case localization.launch.py \
      params_file:=$PWD/src/localization_case/config/amcl-defect.yaml \
      map:=$PWD/.course-kit/v1/fixtures/pr08/map.yaml
    ros2 bag play fixtures/pr07-sensors        # a few seconds
    # EXPECTED: amcl logs "Could not transform odom -> chassis" style
    # lookup error; tf2_echo map odom times out (no map->odom publisher);
    # stop the localization launch BEFORE the player.

Expected failure line to capture here:
    [ERROR] ...: "chassis" passed to lookupTransform argument ... does not exist

## Fixed run (amcl.yaml: base_frame_id: base_link)

STATUS: pending live run. After restoring base_link the SAME command
must print the map->odom translation (AMCL actively publishing), and
the measurement window must show >= 95% availability and covariance
median <= 0.25 m^2 (evidence/pr08/localization.json).

## Stamp correlation note (to verify on the live run)

LaserScan stamps, Odometry stamps and /clock come from the same
simulation timeline of the PR07 recording (bag written with
--use-sim-time); the map->odom lookup uses Time() = latest available.
