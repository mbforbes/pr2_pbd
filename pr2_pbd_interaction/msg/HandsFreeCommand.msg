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
string MOVE_ABSOLUTE_DIRECTION = move_abs

# 'Normal' Commands:
string OPEN = open
string CLOSE = close


# Different argument options that can appear in the field 'args'
# ----------------------------------------------------------------------
# Side
string LEFT_HAND = left_hand
string RIGHT_HAND = right_hand

# Absolute directions
string UP = up
string DOWN = down
string TO_LEFT = to_left
string TO_RIGHT = to_right
string FORWARD = forward
string BACKWARD = backward

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
