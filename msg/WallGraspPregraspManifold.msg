# -----------------------------------------------------------------------------
# Pregrasp manifold for combined wall grasp call.
# -----------------------------------------------------------------------------
# The key feature is that position is sampled, and orientation is
# derived from position.

# -----------------------------------------------------------------------------
# Parameters.
# -----------------------------------------------------------------------------
# Defines the sampling of the positional part.
# Positions of sampled frames will lie in the X-Y plane of position_frame.
geometry_msgs/Pose position_frame

# Defines the desired palm frame orientation.
# During sampling, this orientation will be further rotated in the
# world X-Y plane to orient the Z axis of the palm frame towards the
# object centroid.
geometry_msgs/Quaternion orientation

# The centroid of the object that is being wall grasped.
geometry_msgs/Point object_centroid

# Width and length describe a box in the X-Y plane of position_frame.
# The coordinates of sampled frames' origins will lie in the range
# [-width/2, width/2] in X coordinate and [-height/2, height/2] in
# Y coordinate in the X-Y plane of position_frame.
float32 width
float32 length
