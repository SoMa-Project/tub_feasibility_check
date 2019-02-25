#!/bin/bash

# This script performs a surface grasp check with a PROB2 in a tabletop scene.
# The target object is
# represented by the green bounding box. This object is a terminating
# collision. All other collisions are prohibited.

# The initial goal pose is not feasible due to the collision with the red
# object. The manifold includes Z rotations of the end effector in the range
# [-1.5, 1.5] and the service is able to sample a feasible goal pose from 
# the manifold.
rosservice call /check_kinematics_tabletop "
initial_configuration: [1.44862328, 0.40142573, 0.75049158, 0, 1.43116999, 0]
goal_pose:
  position: {x: 0, y: 0.52, z: 0.190}
  orientation: {x: 0.997, y: 0, z: 0.071, w: 0}
table_pose:
  position:
    x: -0.0985574275255
    y: 0.0364626981318
    z: 0.718236863613
  orientation:
    x: -0.227255776525
    y: 0.0417324751616
    z: 0.255886077881
    w: 0.938688218594
table_from_edges: true
edge_frames:
-
  position:
    x: 0.23587718606
    y: 0.0118581131101
    z: 0.912215173244
  orientation:
    x: -0.199806234162
    y: 0.116035575477
    z: 0.557890539934
    w: 0.797101895436
-
  position:
    x: 0.0669989585876
    y: -0.159065335989
    z: 0.990766584873
  orientation:
    x: -0.23079088776
    y: -0.0110614857981
    z: 0.0356289074289
    w: 0.972287924094
-
  position:
    x: -0.226289525628
    y: -0.112215168774
    z: 0.954806029797
  orientation:
    x: -0.224340699888
    y: -0.0552995889972
    z: -0.152278669899
    w: 0.960949765879
goal_manifold_frame:
  position: {x: 0, y: 0.52, z: 0.196}
  orientation: {x: 0.997, y: 0, z: 0.071, w: 0}
goal_manifold_orientation: {x: 0.997, y: 0, z: 0.071, w: 0}
bounding_boxes_with_poses:
- box:
    type: 0
    dimensions: [0.08, 0.08, 0.08]
  pose:
    position: {x: 0, y: 0.52, z: 0.196}
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
- box:
    type: 0
    dimensions: [0.08, 0.08, 0.05]
  pose:
    position: {x: 0.12, y: 0.51, z: 0.185}
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
- box:
    type: 0
    dimensions: [0.04, 0.15, 0.04]
  pose:
    position: {x: 0.2, y: 0.5, z: 0.190}
    orientation: {x: 0, y: 0, z: 0.3826834, w: 0.9238795}
min_position_deltas: [-0.05, -0.05, -0.07]
max_position_deltas: [0.05, 0.05, 0.05]
min_orientation_deltas: [0, 0, -1.5]
max_orientation_deltas: [0, 0, 1.5]
allowed_collisions:
- {type: 1, box_id: 0, terminating: true}
" 
