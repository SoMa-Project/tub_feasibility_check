#!/bin/bash

# This script performs a surface grasp check. The target object is
# represented by the green bounding box. This object is a terminating
# collision. All other collisions (except the bottom of the container) are prohibited.

# This will check an easy go down movement. Collisions are only allowed with 
# the bottom of the container and the object (collision is required). This movement
# is feasible and thus the planner does not have to generate an alternative 
# trajectory.
rosservice call /check_kinematics_tabletop "
initial_configuration: [-0.4857, 0.4931, 1.259, 1.604, 1.786, -1.804]
goal_pose:
  position: [0.498, 0.129, 0.164]
  orientation: {x: 0.685114499881, y: 0.663267754151, z: 0.217253436334, w: -0.208554435956}
table_surface_pose:
  position: [0.498, 0.129, 0.134]
  orientation: [0.000, 0.000, -0.167, 0.986]
bounding_boxes_with_poses:
- box: 
    type: 1
    dimensions: [0.11955147981643677, 0.0790453553199768, 0.06020838022232056]
  pose: 
    position: [0.498, 0.129, 0.164]
    orientation: {x: 0.000448179713371, y: -0.00122113167373, z: -0.0165826804441, w: 0.999861651771}
goal_manifold_frame:
    position: [0.498, 0.129, 0.134]
    orientation: {x: 0.000448179713371, y: -0.00122113167373, z: -0.0165826804441, w: 0.999861651771}
goal_manifold_orientation: {x: 0.685114499881, y: 0.663267754151, z: 0.217253436334, w: -0.208554435956}
min_position_deltas: [-0.04, -0.07, 0.03]
max_position_deltas: [0.04, -0.01, 0.07]
min_orientation_deltas: [0, 0, -0.5]
max_orientation_deltas: [0, 0, 0.5]
allowed_collisions:
- {type: 1, box_id: 0, terminating: true}
- {type: 2, constraint_name: 'bottom', terminating: false}
"
