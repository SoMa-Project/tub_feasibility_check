# This message type specifies an allowed collision for the CheckKinematics
# service. There are two principal types of allowed collisions, based on
# the value of the "terminating" flag.

# Available specification types.
# Collision with a bounding box. The box_id should be set.
uint8 BOUNDING_BOX=1
# Collision with an environmental constraint. constraint_name should be set.
uint8 ENV_CONSTRAINT=2

# Whether this message allows collision with BOUNDING_BOX or ENV_CONSTRAINT.
uint8 type

# If type is BOUNDING_BOX, then this specifies the bounding box id.
uint32 box_id

# If type is ENV_CONSTRAINT, then this specifies the environmental
# constraint name. Available choices are: "north", "south", "east",
# "west" for the walls of the IFCO and "bottom" for the bottom of the
# IFCO.
string constraint_name

# When terminating is false, then collision of the any robot part with the
# specified object or environmental constraint will be ignored, even if
# the robot part colliding with the object or environmental constraint
# is unsensorized.
#
# When terminating is true, then a collision with a sensorized part of
# the robot will end the check. A collision with an unsensorized part
# of the robot on the other hand will result in failure.
bool terminating

bool required
