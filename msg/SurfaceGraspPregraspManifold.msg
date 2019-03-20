uint8 CIRCULAR=0
uint8 ELONGATED=1

uint8 type

# These parameters are shared between different manifolds.
geometry_msgs/Pose position_frame
geometry_msgs/Point object_centroid
geometry_msgs/Quaternion orientation

float32 orientation_delta
bool orient_outward

# Parameters specific to the circular manifold.

float32 min_radius
float32 max_radius

# Parameters specific to the elongated manifold.
float32 stripe_width
float32 stripe_height
float32 stripe_offset
