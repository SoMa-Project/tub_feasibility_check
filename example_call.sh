#!/bin/bash

rosservice call /check_kinematics "
initial_configuration: [0, 0.1, 0, 2.3, 0, 0.5, 0]
goal_pose:
  position: {x: 0.391, y: -0.3, z: 0.4}
  orientation: {x: 0.997, y: 0, z: 0.071, w: 0}
ifco_pose:
  position: {x: -0.12, y: 0, z: 0.1}
  orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1}
bounding_boxes_with_poses:
- box:
    type: 0
    dimensions: [0.16, 0.16, 0.18]
  pose:
    position: {x: 0.45, y: -0.02, z: 0.3}
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
- box:
    type: 0
    dimensions: [0.16, 0.16, 0.18]
  pose:
    position: {x: 0.5, y: -0.2, z: 0.3}
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
position_deltas: [0.05, 0.25, 0.05]
orientation_deltas: [0, 0, 0]
allowed_collisions:
- {type: 1, box_id: 1, terminate_on_collision: false}
- {type: 1, box_id: 0, terminate_on_collision: false}
" 
