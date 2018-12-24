#!/bin/bash

rosservice call /cerrt_example "
initial_configuration: [0.1, 0.1, 0, 2.3, 0, 0.5, 0]
goal_pose:
  position: {x: 0.45, y: 0.10, z: 0.45}
  orientation: {x: 0.997, y: 0, z: 0.071, w: 0}
ifco_pose:
  position: {x: -0.12, y: 0, z: 0.1}
  orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1}
"
