#!/bin/bash

# This script performs a surface grasp check. The target object is
# represented by the green bounding box. This object is a terminating
# collision. All other collisions are prohibited.

# The initial goal pose is not feasible due to the collision with the red
# object. The manifold includes all possible Z rotations of the end effector, 
# and the service is able to sample a feasible goal pose from the manifold.
rosservice call /check_kinematics "
initial_configuration: [0, 0.1, 0, 2.3, 0, 0.5, 0]
goal_pose:
  position: {x: 0.4, y: -0.02, z: 0.25}
  orientation: {x: 0.997, y: 0, z: 0.071, w: 0}
ifco_pose:
  position: {x: -0.12, y: 0, z: 0.1}
  orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1}
bounding_boxes_with_poses:
- box:
    type: 0
    dimensions: [0.08, 0.08, 0.08]
  pose:
    position: {x: 0.4, y: -0.02, z: 0.25}
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
- box:
    type: 0
    dimensions: [0.08, 0.08, 0.05]
  pose:
    position: {x: 0.52, y: -0.01, z: 0.235}
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
- box:
    type: 0
    dimensions: [0.04, 0.15, 0.04]
  pose:
    position: {x: 0.6, y: 0.2, z: 0.24}
    orientation: {x: 0, y: 0, z: 0.3826834, w: 0.9238795}
position_deltas: [0.01, 0.01, 0.01]
orientation_deltas: [0, 0, 3.14]
allowed_collisions:
- {type: 1, box_id: 0, terminate_on_collision: true}
" 
