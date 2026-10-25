# PR06: frame contract of the vehicle tree

Every transform below has exactly one publisher. Fixed joints go to
`/tf_static` (RELIABLE + TRANSIENT_LOCAL, latched), wheel joints go to
`/tf` but only appear when somebody publishes `/joint_states`.

Default parameters: wheelbase = 0.8 m, track = 0.6 m, wheel_radius = 0.12 m,
body = 0.9 x 0.5 x 0.15 m. Units are SI, axes follow REP-103
(+x forward, +y left, +z up).

| Joint                  | Parent     | Child                  | Type       | Expected transform (xyz, m)        | Owner / channel                                      |
|------------------------|------------|------------------------|------------|------------------------------------|------------------------------------------------------|
| camera_joint           | base_link  | camera_link            | fixed      | (0.40, 0.00, 0.105)                | robot_state_publisher -> tf_static                   |
| lidar_joint            | base_link  | lidar_link             | fixed      | (0.00, 0.00, 0.115)                | robot_state_publisher -> tf_static                   |
| imu_joint              | base_link  | imu_link               | fixed      | (0.10, 0.00, 0.020)                | robot_state_publisher -> tf_static                   |
| front_left_wheel_joint | base_link  | front_left_wheel_link  | continuous | (+wheelbase/2, +track/2, 0) = (0.4, 0.3, 0)   | robot_state_publisher -> tf (needs /joint_states)    |
| front_right_wheel_joint| base_link  | front_right_wheel_link | continuous | (+wheelbase/2, -track/2, 0) = (0.4, -0.3, 0)  | robot_state_publisher -> tf (needs /joint_states)    |
| rear_left_wheel_joint  | base_link  | rear_left_wheel_link   | continuous | (-wheelbase/2, +track/2, 0) = (-0.4, 0.3, 0)  | robot_state_publisher -> tf (needs /joint_states)    |
| rear_right_wheel_joint | base_link  | rear_right_wheel_link  | continuous | (-wheelbase/2, -track/2, 0) = (-0.4, -0.3, 0) | robot_state_publisher -> tf (needs /joint_states)    |

Notes:

- `base_link` is the root of the tree; every link above is reachable from
  it, and no link has two parents (checked by test_model.py).
- The wheel joints are `continuous` (rolling has no position limits);
  their axes point along +y (the axle direction).
- In this PR nobody publishes `/joint_states`, so `tf2_echo` and
  `view_frames` show only the three fixed sensor frames - exactly the
  three rows that live on tf_static. The wheel rows exist in the URDF
  and in this table, but not yet in the live tf tree.

## Live confirmation (ros2 launch course_description description.launch.py)

    ## tf2_echo base_link lidar_link
    At time 0.0
    - Translation: [0.000, 0.000, 0.115]
    - Rotation: in Quaternion (xyzw) [0.000, 0.000, 0.000, 1.000]

    ## tf2_echo base_link camera_link
    At time 0.0
    - Translation: [0.400, 0.000, 0.105]
    - Rotation: in Quaternion (xyzw) [0.000, 0.000, 0.000, 1.000]

    ## view_frames (frame graph)
    camera_link: parent: 'base_link', broadcaster: 'default_authority'
    imu_link:    parent: 'base_link', broadcaster: 'default_authority'
    lidar_link:  parent: 'base_link', broadcaster: 'default_authority'
