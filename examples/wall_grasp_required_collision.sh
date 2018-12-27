#!/bin/bash

# CURRENTLY REQUIRES TIME TO ADAPT SITUATION TO THE NEW HAND

# This script demonstrates the required collision functionality.
# The first check finds a sampled pose that is not behind the green object.
# The subsequent check is not able to find a feasible target pose such
# that the green object is collided with on the way.
result=$(rosservice call /check_kinematics "
initial_configuration: [0.1, 0.1, 0, 2.3, 0, 0.5, 0]
goal_pose:
  position: {x: 0.4, y: -0.1, z: 0.3}
  orientation: {x: 0.6830127, y: -0.6830127, z: 0.1830127, w: 0.1830127}
ifco_pose:
  position: {x: -0.12, y: 0, z: 0.1}
  orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1}
bounding_boxes_with_poses:
- box:
    type: 0
    dimensions: [0.06, 0.06, 0.06]
  pose:
    position: {x: 0.4, y: -0.02, z: 0.24}
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
- box:
    type: 0
    dimensions: [0.06, 0.06, 0.06]
  pose:
    position: {x: 0.4, y: -0.2, z: 0.24}
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
min_position_deltas: [0, 0.0, 0.0]
max_position_deltas: [0.4, 0.1, 0.0]
min_orientation_deltas: [0, 0, 0]
max_orientation_deltas: [0, 0, 0]
allowed_collisions:
- {type: 2, constraint_name: 'bottom', terminate_on_collision: true}
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
  position: {x: 0.45, y: -0.4, z: 0.35}
  orientation: {x: 0.4909103, y: -0.3602125, z: 0.6225606, w: 0.4916018}
ifco_pose:
  position: {x: -0.12, y: 0, z: 0.1}
  orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1}
bounding_boxes_with_poses:
- box:
    type: 0
    dimensions: [0.06, 0.06, 0.06]
  pose:
    position: {x: 0.4, y: -0.02, z: 0.24}
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
- box:
    type: 0
    dimensions: [0.06, 0.06, 0.06]
  pose:
    position: {x: 0.4, y: -0.2, z: 0.24}
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
min_position_deltas: [0, 0, 0]
max_position_deltas: [0.2, 0, 0]
min_orientation_deltas: [0, 0, 0]
max_orientation_deltas: [0, 0, 0]
allowed_collisions:
- {type: 2, constraint_name: 'bottom', terminate_on_collision: false}
- {type: 1, box_id: 0, terminate_on_collision: false, required_collision: true}
- {type: 2, constraint_name: 'south', terminate_on_collision: true}
" 
