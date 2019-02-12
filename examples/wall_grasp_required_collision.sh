#!/bin/bash

# This script demonstrates the required collision functionality.
# First check is placing the hand behind box_0, terminating on contact with
# ifco bottom. The second one is "sliding" box_0 towards the wall.

result=$(rosservice call /check_kinematics "
initial_configuration: [0.1, 0.1, 0, 2.3, 0, 0.5, 0]
goal_pose:
  position: {x: 0.4, y: -0.02, z: -0.096}
  orientation: {x: 0.997, y: 0, z: 0.071, w: 0}
ifco_pose:
  position: {x: 0.48, y: 0, z: -0.146}
  orientation: {x: 0.0, y: 0.0, z: 0.7071081, w: 0.7071055}
goal_manifold_frame:
  position: {x: 0.3, y: 0.1, z: -0.096}
  orientation: {x: 0.997, y: 0, z: 0.071, w: 0}
goal_manifold_orientation: {x: -0.658703, y: -0.691938, z: 0.2037607, w: -0.2140415}
bounding_boxes_with_poses:
- box:
    type: 0
    dimensions: [0.06, 0.06, 0.06]
  pose:
    position: {x: 0.4, y: -0.02, z: -0.111}
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
- box:
    type: 0
    dimensions: [0.06, 0.06, 0.06]
  pose:
    position: {x: 0.4, y: -0.2, z: -0.111}
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
min_position_deltas: [0, 0.0, -0.08]
max_position_deltas: [0.2, 0.1, 0.01]
min_orientation_deltas: [-0.1, -0.1, -0.1]
max_orientation_deltas: [0.1, 0.1, 0.1]
allowed_collisions:
- {type: 2, constraint_name: 'bottom', terminating: true, required: true}
")


echo $result

final_configuration=$(echo $result | perl -n -e'/final_configuration: (\[[^]]*\])/ && print $1')
if [ "$final_configuration" == '[]' ]; then
    echo "The first check failed, rerun the script"
    exit
fi

rosservice call /check_kinematics "
initial_configuration: $final_configuration
goal_pose:
  position: {x: 0.4, y: -0.1, z: -0.096}
  orientation: {x: -0.658703, y: -0.691938, z: 0.2037607, w: -0.2140415}
ifco_pose:
  position: {x: 0.48, y: 0, z: -0.146}
  orientation: {x: 0.0, y: 0.0, z: 0.7071081, w: 0.7071055}
goal_manifold_frame:
  position: {x: 0.4, y: -0.23, z: -0.01}
  orientation: {x: 0.997, y: 0, z: 0.071, w: 0}
goal_manifold_orientation: {x: -0.658703, y: -0.691938, z: 0.2037607, w: -0.2140415}
bounding_boxes_with_poses:
- box:
    type: 0
    dimensions: [0.06, 0.06, 0.06]
  pose:
    position: {x: 0.4, y: -0.02, z: -0.111}
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
- box:
    type: 0
    dimensions: [0.06, 0.06, 0.06]
  pose:
    position: {x: 0.4, y: -0.2, z: -0.111}
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
min_position_deltas: [0, 0, 0]
max_position_deltas: [0.2, 0.1, 0]
min_orientation_deltas: [-0.1, -0.1, -0.1]
max_orientation_deltas: [0.1, 0.1, 0.1]
allowed_collisions:
- {type: 2, constraint_name: 'bottom'}
- {type: 1, box_id: 0, required: true}
- {type: 2, constraint_name: 'south', terminating: true, required: true}
"
