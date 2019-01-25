#!/bin/bash

# PreGrasp
result=$(rosservice call /check_kinematics "
initial_configuration: [0.457929, 0.295013, -0.232804, 2.0226, 0.2, 0.1, 0.1]
goal_pose:
  position: {x: 0.547870665859, y: -0.161491472467, z: 0.579824850577}
  orientation: {x: -0.56130049162, y: 0.560337402091, z: 0.43005324755, w: 0.431297992341}
ifco_pose:
  position: {x: 0.500174838871, y: -0.0999421447411, z: 0.280010306471}
  orientation: {x: -0.000513912280661, y: -0.00105990230508, z: 0.707109448793, w: 0.707103132456}
bounding_boxes_with_poses:
- box:
    type: 1
    dimensions: [0.11954766511917114, 0.0790407657623291, 0.06298935413360596]
  pose:
    position: {x: 0.548381108793, y: -0.0113139192822, z: 0.349941310567}
    orientation: { x: 0.000404456403294, y: -0.00110629055252, z: -0.0165607548033, w: 0.999862167469}
min_position_deltas: [-0.05, -0.05, -0.05]
max_position_deltas: [0.05, 0.06, 0.05]
min_orientation_deltas: [0, 0, -0.17]
max_orientation_deltas: [0, 0, 0.17]
allowed_collisions: []
"
)

echo $result

final_configuration=$(echo $result | perl -n -e'/final_configuration: (\[[^]]*\])/ && print $1')

echo $final_configuration


# The second check starts from the final configuration of the first check.
# GoDown
rosservice call /check_kinematics "
initial_configuration: $final_configuration
goal_pose:
  position: {x: 0.547870665859, y: -0.161491472467, z: 0.329824850577}
  orientation: {x: -0.56130049162, y: 0.560337402091, z: 0.43005324755, w: 0.431297992341}
ifco_pose:
  position: {x: 0.500174838871, y: -0.0999421447411, z: 0.280010306471}
  orientation: {x: -0.000513912280661, y: -0.00105990230508, z: 0.707109448793, w: 0.707103132456}
bounding_boxes_with_poses:
- box:
    type: 1
    dimensions: [0.11954766511917114, 0.0790407657623291, 0.06298935413360596]
  pose:
    position: {x: 0.548381108793, y: -0.0113139192822, z: 0.349941310567}
    orientation: { x: 0.000404456403294, y: -0.00110629055252, z: -0.0165607548033, w: 0.999862167469}
min_position_deltas: [-0.05, -0.05, -0.05]
max_position_deltas: [0.08, 0.05, 0.05]
min_orientation_deltas: [0, 0, 0.17]
max_orientation_deltas: [0, 0, 0.17]
allowed_collisions:
- {type: 2, constraint_name: 'bottom', terminating: false}
" 

echo $result

final_configuration=$(echo $result | perl -n -e'/final_configuration: (\[[^]]*\])/ && print $1')


# LiftHand
rosservice call /check_kinematics "
initial_configuration: $final_configuration
goal_pose:
  position: {x: 0.547870665859, y: -0.161491472467, z: 0.429824850577}
  orientation: {x: -0.56130049162, y: 0.560337402091, z: 0.43005324755, w: 0.431297992341}
ifco_pose:
  position: {x: 0.500174838871, y: -0.0999421447411, z: 0.280010306471}
  orientation: {x: -0.000513912280661, y: -0.00105990230508, z: 0.707109448793, w: 0.707103132456}
bounding_boxes_with_poses:
- box:
    type: 1
    dimensions: [0.11954766511917114, 0.0790407657623291, 0.06298935413360596]
  pose:
    position: {x: 0.548381108793, y: -0.0113139192822, z: 0.349941310567}
    orientation: { x: 0.000404456403294, y: -0.00110629055252, z: -0.0165607548033, w: 0.999862167469}
min_position_deltas: [-0.05, -0.05, -0.05]
max_position_deltas: [0.05, 0.05, 0.05]
min_orientation_deltas: [0, 0, 0.17]
max_orientation_deltas: [0, 0, 0.17]
allowed_collisions:
- {type: 2, constraint_name: 'bottom', terminating: false}
" 

echo $result

final_configuration=$(echo $result | perl -n -e'/final_configuration: (\[[^]]*\])/ && print $1')

# SlideToWall
rosservice call /check_kinematics "
initial_configuration: $final_configuration
goal_pose:
  position: {x: 0.547866748325, y: 0.238508408272, z: 0.430133708549}
  orientation: {x: -0.56130049162, y: 0.560337402091, z: 0.43005324755, w: 0.431297992341}
ifco_pose:
  position: {x: 0.500174838871, y: -0.0999421447411, z: 0.280010306471}
  orientation: {x: -0.000513912280661, y: -0.00105990230508, z: 0.707109448793, w: 0.707103132456}
bounding_boxes_with_poses:
- box:
    type: 1
    dimensions: [0.11954766511917114, 0.0790407657623291, 0.06298935413360596]
  pose:
    position: {x: 0.548381108793, y: -0.0113139192822, z: 0.349941310567}
    orientation: { x: 0.000404456403294, y: -0.00110629055252, z: -0.0165607548033, w: 0.999862167469}
min_position_deltas: [-0.05, -0.05, -0.05]
max_position_deltas: [0.05, 0.05, 0.05]
min_orientation_deltas: [0, 0, 0.17]
max_orientation_deltas: [0, 0, 0.17]
allowed_collisions:
- {type: 2, constraint_name: 'bottom', terminating: false}
- {type: 2, constraint_name: 'north', terminating: true}
- {type: 1, box_id: 0, terminating: false}
" 
