uint8 CIRCULAR=0
uint8 ELONGATED=1

uint8 type

# These parameters are shared between different manifolds.
geometry_msgs/Pose initial_frame
float32 orientation_delta

# Parameters specific to the circular manifold.

# The sampled frame origin is sampled in the plane defined by the X and Y axes of the initial frame.
# It is sampled uniformly from a circle lying in that plane around the initial frame position with a certain radius.
# The sampled frame orientation is rotated around Z axis such that the X axis of the frame points towards
# the origin of the initial frame. On top of that, a sampled rotation around Z from the range of
# [-orientation_delta, orientation_delta] is applied.

float32 radius

# Parameters specific to the elongated manifold.
float32 stripe_width
float32 stripe_height
float32 stripe_offset
