#!/bin/bash

# This script does two successful checks that do not trigger contact manifold
# sampling. The bounding box of the grasped object is colored green.
# All objects that are not explicitly listed are forbidden collisions.

# A terminating collision is an object that can be touched by the sensorized
# parts of the robot. A collision with such an object terminates planning.

# A ignored collision is an object that can be touched by both sensorized
# and unsensorized parts of the robot without triggering a failure.

# A required collision determines the outcome of planning. The outcome is
# successful only when all required collisions were present during the trajectory.

# In the first check, the end effector goes down to the pregrasp position.
# Bottom of the ifco is a terminating collision.
result=$(rosservice call /check_kinematics "
initial_configuration: [0.1, 0.1, 0, 2.3, 0, 0.5, 0]
goal_pose:
  position: {x: 0.45, y: 0.10, z: 0.004}
  orientation: {x: 0.6830127, y: 0.6830127, z: -0.1830127, w: 0.1830127}
ifco_pose:
  position: {x: 0.48, y: 0, z: -0.146}
  orientation: {x: 0.0, y: 0.0, z: 0.7071081, w: 0.7071055}
goal_manifold_frame:
  position: {x: 0.45, y: 0.10, z: 0.004}
  orientation: {x: 0.6830127, y: 0.6830127, z: -0.1830127, w: 0.1830127}
goal_manifold_orientation: {x: 0.6830127, y: 0.6830127, z: -0.1830127, w: 0.1830127}
bounding_boxes_with_poses:
- box:
    type: 0
    dimensions: [0.09, 0.09, 0.09]
  pose:
    position: {x: 0.45, y: -0.02, z: -0.046}
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
- box:
    type: 0
    dimensions: [0.08, 0.08, 0.08]
  pose:
    position: {x: 0.6, y: -0.2, z: -0.046}
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
min_position_deltas: [-0.05, -0.05, -0.05]
max_position_deltas: [0.05, 0.05, 0.05]
min_orientation_deltas: [0, 0, 0]
max_orientation_deltas: [0, 0, 0]
allowed_collisions:
- {type: 2, constraint_name: 'bottom', terminating: true}
"
)

echo $result

final_configuration=$(echo $result | perl -n -e'/final_configuration: (\[[^]]*\])/ && print $1')

# The second check starts from the final configuration of the first check.
# The end effector simulates the sliding motion.
# Grasped object and bottom of the ifco are ignored collisions.
# South wall of the ifco is terminating collision.
rosservice call /check_kinematics "
initial_configuration: $final_configuration
goal_pose:
  position: {x: 0.45, y: -0.4, z: 0.004}
  orientation: {x: 0.4909103, y: -0.3602125, z: 0.6225606, w: 0.4916018}
ifco_pose:
  position: {x: 0.48, y: 0, z: -0.146}
  orientation: {x: 0.0, y: 0.0, z: 0.7071081, w: 0.7071055}
goal_manifold_frame:
  position: {x: 0.45, y: -0.2, z: 0.004}
  orientation: {x: 0.4909103, y: -0.3602125, z: 0.6225606, w: 0.4916018}
goal_manifold_orientation: {x: 0.4909103, y: -0.3602125, z: 0.6225606, w: 0.4916018}
bounding_boxes_with_poses:
- box:
    type: 0
    dimensions: [0.09, 0.09, 0.09]
  pose:
    position: {x: 0.45, y: -0.02, z: -0.046}
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
- box:
    type: 0
    dimensions: [0.08, 0.08, 0.08]
  pose:
    position: {x: 0.6, y: -0.2, z: -0.046}
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
min_position_deltas: [-0.2, -0.2, -0.2]
max_position_deltas: [0.2, 0.2, 0.2]
min_orientation_deltas: [0, 0, 0]
max_orientation_deltas: [0, 0, 0]
allowed_collisions:
- {type: 2, constraint_name: 'bottom'}
- {type: 1, box_id: 0}
- {type: 2, constraint_name: 'south', terminating: true}
" 
