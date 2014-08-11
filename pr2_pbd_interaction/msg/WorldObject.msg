# WorldObject.msg: An observed object of the world.

# Constants
# ======================================================================
# NOTE(mbforbes): *MUST* keep these up-to-date with commands.yml. In
# fact, this file should really be generated from that.

# Colors:
string RED = red
string BLUE = blue
string GREEN = green
string UNKNOWN = unknown

# Object types:
string BOX = box
string CUP = cup
# UNKNOWN also applies here


# Fields
# ======================================================================

# Basic info
string name
string color
string type

# Reachability arrays have two entires: right, left
bool[] is_pickupable

bool[] is_above_reachable
bool[] is_leftof_reachable
bool[] is_rightof_reachable
bool[] is_frontof_reachable
bool[] is_behind_reachable
bool[] is_topof_reachable
bool[] is_nextto_reachable
bool[] is_near_reachable

# Computed w.r.t other objects
bool is_leftmost
bool is_rightmost
bool is_farthest
bool is_nearest
bool is_biggest
bool is_smallest
