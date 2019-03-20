# -----------------------------------------------------------------------------
# Pregrasp manifold for combined surface grasp call.
# -----------------------------------------------------------------------------
# The key feature is that position is sampled from a defined region,
# and orientation is derived from position.

# There are two types of manifolds available. They differ in the way
# the position is sampled, but the orientation is derived from position
# in the same manner for both of them.

uint8 CIRCULAR=0
uint8 ELONGATED=1

# The type of the manifold. Must be either CIRCULAR (0) or ELONGATED(1).
uint8 type

# -----------------------------------------------------------------------------
# The following parameters are shared between different manifolds.
# -----------------------------------------------------------------------------
# Defines the sampling of the positional part.
# Positions of sampled frames will lie in the X-Y plane of position_frame.
geometry_msgs/Pose position_frame

# Defines the desired palm frame orientation.
# During sampling, this orientation will be further rotated in the
# world X-Y plane to orient the X axis of the palm frame towards the
# object centroid.
geometry_msgs/Quaternion orientation

# The centroid of the object that is being surface grasped.
geometry_msgs/Point object_centroid

# Defines additional slack in orientation, that will be randomly
# applied after the palm frame is oriented towards the object centroid.
# The value of rotation will be sampled from the range
# [-orientation_delta, orientation_delta].
float32 orientation_delta

# Whether to orient the X axis of the palm frame towards the
# object centroid, or away from it.
bool orient_outward

# -----------------------------------------------------------------------------
# Circular manifold.
# -----------------------------------------------------------------------------
# The positions of the sampled frames will lie in the X-Y plane of position_frame
# and will have distance to the origin of that frame in the range
# [min_radius, max_radius].

float32 min_radius
float32 max_radius

# -----------------------------------------------------------------------------
# Elongated manifold.
# -----------------------------------------------------------------------------
# The elongated manifold consists of two stripes that are oriented
# along the X axis of position_frame. One of them lies in the half-plane with
# positive Y and another in the half-plane with negative Y.

# The width of stripes. Sampled frames X coordinates will lie in the range
# [-stripe_width/2, stripe_width/2] in position_frame.
float32 stripe_width

# stripe_height and stripe_offset together define the region of sampling
# for Y coordinate. It will be the union of
# [-stripe_offset - stripe_height, -stripe_offset] with
# [stripe_offset, stripe_offset + stripe_height].
float32 stripe_height
float32 stripe_offset
