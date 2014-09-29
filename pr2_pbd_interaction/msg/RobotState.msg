# RobotState.msg: Information about the state of the robot.

# Constants
# ======================================================================
# NOTE(mbforbes): *MUST* keep these up-to-date with constants in
# handsfree-parser.

# Last CMD side
string LEFT_HAND = left_hand
string RIGHT_HAND = right_hand
string NEITHER = neither

# Gripper states:
string OPEN = open
string CLOSED_EMPTY = closed_empty
string HAS_OBJ = has_obj


# Fields
# ======================================================================

string last_cmd_side
string last_referred_obj_name
bool is_executing

# Arrays have two entires: right, left
string[] gripper_states
bool[] can_move_up
bool[] can_move_down
bool[] can_move_toleft
bool[] can_move_toright
bool[] can_move_forward
bool[] can_move_backward
