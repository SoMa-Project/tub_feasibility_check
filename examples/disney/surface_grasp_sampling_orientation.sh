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
  position: {x: 0.18, y: 0.6, z: 0.146}
  orientation: {x: 0.0, y: 0.0, z: 0.7071068, w: 0.7071068}
edge_frames:
- position:
    x: -0.362667769194
    y: 0.0230741873384
    z: 0.833642959595
  orientation:
    x: -0.105323456787
    y: -0.157057728398
    z: -0.732987024524
    w: 0.653429294557
- position:
    x: 0.0158419162035
    y: 0.21796926856
    z: 0.777151107788
  orientation:
    x: -0.0319034574641
    y: 0.186392955899
    z: 0.981449229674
    w: 0.0315796422009
- position:
    x: 0.33892339468
    y: -0.0316337645054
    z: 0.89374423027
  orientation:
    x: -0.156191844284
    y: 0.106603363431
    z: 0.659397143631
    w: 0.727623056517
- position:
    x: -0.0395862907171
    y: -0.226528853178
    z: 0.950236082077
  orientation:
    x: -0.185376567614
    y: -0.0373589779824
    z: -0.0603642594255
    w: 0.980099984842
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
