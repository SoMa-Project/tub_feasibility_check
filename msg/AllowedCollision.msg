# This message type specifies an allowed collision for the CheckKinematics
# service.

# Available types of allowed collisions.
# Collision with a bounding box.
uint8 BOUNDING_BOX=1
# Collision with an environmental constraint.
uint8 ENV_CONSTRAINT=2

# The type of allowed collision.
uint8 type

# If type=BOUNDING_BOX, then this specifies the bounding box id.
uint32 box_id

# If type=ENV_CONSTRAINT, then this specifies the environmental
# constraint name. Available choices are: "north", "south", "east",
# "west" for the walls of the IFCO and "bottom" for the bottom of the
# IFCO.
string constraint_name

# If true, this collision will cause the planner to stop and output
# success if all other collisions in the current step are allowed.
# If false, this collision will be ignored by the planner.
bool terminate_on_collision

# If true, planner will not output success if this collision did not
# happen at some point.
bool required_collision
