#!/bin/bash

# This script performs a surface grasp check. The target object is
# represented by the green bounding box. This object is a terminating
# collision. All other collisions (except the bottom of the container) are prohibited.

# This will check an easy go down movement. Collisions are only allowed with 
# the bottom of the container and the object (collision is required). This movement
# is feasible and thus the planner does not have to generate an alternative 
# trajectory.
rosservice call /check_kinematics "
initial_configuration: [-0.5945735614121268, 0.42180752185581194, 0.5116913273945006, 1.592272893703166, 0.424190933949368, 1.2527738218055204, -1.7908341943112134]
goal_pose:
  position: {x: 0.524368202392, y: -0.110797924543, z: 0.301181716182}
  orientation: {x: 0.685114499881, y: 0.663267754151, z: 0.217253436334, w: -0.208554435956}
ifco_pose:
  position: {x: 0.500191347818, y: -0.0999358441943, z: 0.28001147817}
  orientation: {x: -0.000566068955935, y: -0.00117116004218, z: 0.707109752686, w: 0.707102613209}
bounding_boxes_with_poses:
- box: 
    type: 1
    dimensions: [0.11955147981643677, 0.0790453553199768, 0.06020838022232056]
  pose: 
    position: {x: 0.548410148325, y: -0.0112594768472, z: 0.351324941801}
    orientation: {x: 0.000448179713371, y: -0.00122113167373, z: -0.0165826804441, w: 0.999861651771}
min_position_deltas: [-0.01, -0.01, -0.01]
max_position_deltas: [0.01, 0.01, 0.01]
min_orientation_deltas: [0, 0, -0.5]
max_orientation_deltas: [0, 0, 0.5]
allowed_collisions:
- {type: 1, box_id: 0, terminate_on_collision: true, ignored_collision: true, required_collision: true}
- {type: 2, constraint_name: 'bottom', terminate_on_collision: false}
"
