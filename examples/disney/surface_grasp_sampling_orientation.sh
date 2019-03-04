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
