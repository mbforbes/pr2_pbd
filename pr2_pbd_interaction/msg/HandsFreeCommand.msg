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

# 'Normal' Commands:
string OPEN = open
string CLOSE = close


# Different argument options that can appear in the field 'args'
# ----------------------------------------------------------------------
# Side
string LEFT_HAND = left_hand
string RIGHT_HAND = right_hand


# Fields
# ======================================================================
string cmd
string[] args
string[] phrases
