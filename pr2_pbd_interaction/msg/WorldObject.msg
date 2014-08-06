# WorldObject.msg: An observed object of the world.

# Constants
# ======================================================================
# NOTE(mbforbes): *MUST* keep these up-to-date with commands.yml. In
# fact, this file should really be generated from that.

# Colors:
string RED = red
string BLUE = blue
string GREEN = green

# Object types:
string BOX = box
string CUP = cup
string UNKNOWN = unknown


# Fields
# ======================================================================

string name

# Reachability arrays have two entires: right, left
bool[] is_pickupable
bool[] is_above_reachable
bool[] is_nextto_reachable

# Color, type, location.
string color
string type
bool is_leftmost
bool is_rightmost
bool is_farthest
bool is_nearest
bool is_biggest
bool is_smallest
