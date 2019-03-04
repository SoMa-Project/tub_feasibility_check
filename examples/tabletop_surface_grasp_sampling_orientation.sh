#!/bin/bash

# This script performs a surface grasp check. The target object is
# represented by the green bounding box. This object is a terminating
# collision. All other collisions are prohibited.

# The initial goal pose is not feasible due to the collision with the red
# object. The manifold includes Z rotations of the end effector in the range
# [-1.5, 1.5] and the service is able to sample a feasible goal pose from 
# the manifold.
rosservice call /check_kinematics_tabletop "
initial_configuration: [0, 0.1, 0, 2.3, 0, 0.5, 0]
goal_pose:
  position: {x: 0.4, y: -0.02, z: -0.096}
  orientation: {x: 0.997, y: 0, z: 0.071, w: 0}
table_pose:
  position: {x: 0.48, y: 0, z: -0.146}
  orientation: {x: 0.0, y: 0.0, z: 0, w: 1}
goal_manifold_frame:
  position: {x: 0.4, y: -0.02, z: -0.096}
  orientation: {x: 0.997, y: 0, z: 0.071, w: 0}
goal_manifold_orientation: {x: 0.997, y: 0, z: 0.071, w: 0}
bounding_boxes_with_poses:
- box:
    type: 0
    dimensions: [0.08, 0.08, 0.08]
  pose:
    position: {x: 0.4, y: -0.02, z: -0.096}
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
- box:
    type: 0
    dimensions: [0.08, 0.08, 0.05]
  pose:
    position: {x: 0.52, y: -0.01, z: -0.111}
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
- box:
    type: 0
    dimensions: [0.04, 0.15, 0.04]
  pose:
    position: {x: 0.6, y: 0.2, z: -0.106}
    orientation: {x: 0, y: 0, z: 0.3826834, w: 0.9238795}
min_position_deltas: [-0.05, -0.05, -0.07]
max_position_deltas: [0.05, 0.05, 0.05]
min_orientation_deltas: [0, 0, -1.5]
max_orientation_deltas: [0, 0, 1.5]
allowed_collisions:
- {type: 1, box_id: 0, terminating: true}
" 
