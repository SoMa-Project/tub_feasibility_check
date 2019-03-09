uint8 SIMPLE_BOX=0
uint8 ORIENTED_RING=1

uint8 type

# Description of the simple box manifold.
geometry_msgs/Pose position_frame

# Coordinate X, Y, Z will be sampled in the goal manifold frame from the range:
# [min_position_deltas[0, 1, 2], max_position_deltas[0, 1, 2]]
# For every i: min_position_deltas[i] <= max_position_deltas[i] should be true
float64[3] min_position_deltas
float64[3] max_position_deltas

# The goal manifold orientation in terms of world frame. It is used for
# sampling the orientational part.
geometry_msgs/Quaternion orientation

# The sampled orientation is in XYZ Euler angles, applied on the top of
# goal_manfifold_orientation
# Angle X, Y, Z will be sampled from the range:
# [min_orientation_deltas[0, 1, 2], max_orientation_deltas[0, 1, 2]]
# For every i: min_orientation_deltas[i] <= max_orientation_deltas[i] should be
# true
# Furthermore, the values cannot be outside of the following intervals:
# 0: (-pi, pi)
# 1: (-pi/2, pi/2)
# 2: (-pi, pi)
float64[3] min_orientation_deltas
float64[3] max_orientation_deltas
