# The sampled frame origin is sampled in the plane defined by the X and Y axes of the initial frame.
# It is sampled uniformly from a circle lying in that plane around the initial frame position with a certain radius.
# The sampled frame orientation is rotated around Z axis such that the X axis of the frame points towards
# the origin of the initial frame. On top of that, a sampled rotation around Z from the range of
# [-orientation_delta, orientation_delta] is applied.

geometry_msgs/Pose initial_frame
float32 radius
float32 orientation_delta
