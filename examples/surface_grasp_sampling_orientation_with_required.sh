#!/bin/bash

# Similar to surface_grasp_sampling_orientation.sh, but here the manifold includes
# more space where the hand does not make contact with box_0, and box_0 is a required
# collision.

rosservice call /check_kinematics "
initial_configuration: [0, 0.1, 0, 2.3, 0, 0.5, 0]
goal_pose:
  position: {x: 0.4, y: -0.02, z: -0.096}
  orientation: {x: 0.997, y: 0, z: 0.071, w: 0}
ifco_pose:
  position: {x: 0.48, y: 0, z: -0.146}
  orientation: {x: 0.0, y: 0.0, z: 0.7071081, w: 0.7071055}
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
min_position_deltas: [-0.05, -0.05, -0.20]
max_position_deltas: [0.05, 0.05, 0.05]
min_orientation_deltas: [0, 0, -1.5]
max_orientation_deltas: [0, 0, 1.5]
allowed_collisions:
- {type: 1, box_id: 0, terminating: true, required: true }
" 
