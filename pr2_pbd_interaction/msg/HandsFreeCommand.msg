# HandsFreeCommand.msg: Commands issued for the hands-free PbD system.

# Constants
# ======================================================================
# NOTE(mbforbes): *MUST* keep these up-to-date with commands.yml. In
# fact, this file should really be generated from that.

# Different 'command' (verb) options for the field 'cmd'.
# ----------------------------------------------------------------------
# 'Admin' commands:
string EXECUTE = execute
string STOP = stop
string SWITCH_ACTION = switch_action
string CREATE_NEW_ACTION = create_new_action
string CLARIFY = clarify

# 'Programming' Commands:
# Action 1
string MOVE_ABSOLUTE_POSE = move_abs_pos
# Action 2
string MOVE_RELATIVE_POSITION = move_rel
# Action 3
string MOVE_ABSOLUTE_DIRECTION = move_abs
# Action 4
string MOVE_RELATIVE_DIRECTION = move_rel_dir
# Action 5
string PLACE_AT_LOCATION = place_loc
# Action 6
string PLACE_RELATIVE = place
# Action 7
string PICKUP = pick_up
# Action 8
string POINT_TO = point_to
# Action 9
string ROTATE = rotate
# Action 10
string LOOK_AT = look_at
# Action 11
string OPEN = open
# Action 12
string CLOSE = close


# Different argument options that can appear in the field 'args'
# ----------------------------------------------------------------------
# Side
string LEFT_HAND = left_hand
string RIGHT_HAND = right_hand
string BOTH_HANDS = both_hands

# Absolute poses
string TO_SIDE = to_side

# Absolute directions
string UP = up
string DOWN = down
string TO_LEFT = to_left
string TO_RIGHT = to_right
string FORWARD = forward
string BACKWARD = backward

# Relative positions
string ABOVE = above
string NEXT_TO = next_to
string TO_LEFT_OF = to_left_of
string TO_RIGHT_OF = to_right_of
string IN_FRONT_OF = in_front_of
string BEHIND = behind
string ON_TOP_OF = on_top_of
string NEAR = near

# Relative directions
string AWAY = away
string TOWARDS = towards

# Rotation directions
string CW = cw
string CCW = ccw

# Which
string NEXT = next
string PREVIOUS = previous

# Action
string ACTION = action

# Fields
# ======================================================================
string cmd
string[] args
string[] phrases
