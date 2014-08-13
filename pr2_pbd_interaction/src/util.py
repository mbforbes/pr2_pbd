'''Utility classes for Hands-Free PbD system. Separate from PbD, living
here now for convenience.'''

__author__ = 'mbforbes'


# ######################################################################
# Imports
# ######################################################################

# Core ROS imports come first.
import roslib
roslib.load_manifest('pr2_pbd_interaction')
import rospy


# ######################################################################
# Module level constants
# ######################################################################

FLOAT_COMPARE_EPSILON = 0.01


# ######################################################################
# Classes
# ######################################################################

class Numbers(object):
    '''Helpful and miscellaneous.'''

    @staticmethod
    def are_floats_close(a, b, epsilon=FLOAT_COMPARE_EPSILON):
        '''Checks whether two floats are within epsilon of each other.

        Args:
            a (float): One number.
            b (float): The other number.
            epsilon (float): Acceptable wiggle room (+/-) between a and
                b.

        Returns:
            bool: Whether a and b are within epsilon of each other.
        '''
        # We try to do this in an overflow-friendly way, though it
        # probably isn't a big deal with our use cases and python.
        return a - epsilon <= b if a > b else b - epsilon <= a


class Options(object):
    '''A safe accessor to options with defaults.'''

    defaults = {}

    def __init__(self, options={}):
        '''
        Args:
            options ({str: object}, optional): Defaults to {}.
        '''
        self.options = options
        # Convenience check to avoid redundant specifications.
        for opt, val in options.iteritems():
            if self.get_default(opt) == val:
                rospy.logwarn(
                    'Specified option same as default. Option: ' + str(opt) +
                    ', Value: ' + str(val) + '.')

    def get(self, option_name):
        '''
        Gets the set option by option_name, or the default if it wasn't
        explicitly set.

        Args:
            option_name (str)

        Returns:
            object
        '''
        if option_name in self.options:
            return self.options[option_name]
        else:
            # Python awesome note: you can call class methods from an
            # instance, and it passes the class (cls) instead of the
            # instance (self). How cool is that?!?
            return self.get_default(option_name)

    @classmethod
    def get_default(cls, option_name):
        '''
        Gets the default value for option_name.

        Args:
            option_name (str)

        Returns:
            object
        '''
        return cls.defaults[option_name]


class GlobalOptions(Options):
    '''Options for the running of the system.'''

    # What options we can specify.
    ABORT_PRE_CHECK = 'abort program execution on precheck failure'
    ABORT_CORE = 'abort program execution on command execution failure'
    ABORT_POST_CHECK = 'abort program execution on postcheck failure'

    # TODO(mbforbes): Load from ROS params!
    defaults = {
        ABORT_PRE_CHECK: True,
        ABORT_CORE: True,
        ABORT_POST_CHECK: True,
    }


class CommandOptions(Options):
    '''How to execute a Command.'''

    # What options we can specify.
    FEEDBACK_PRE_CHECK_PROG = (
        'notify user when precheck fails during programming')
    PRE_CHECK_FATAL = (
        'whether the execution should normally be stopped if pre-check fails')
    FEEDBACK_PRE_CHECK_EXEC = (
        'notify user when precheck fails during execution')
    NARRATE_PROG = 'narrate commands during programming'
    NARRATE_EXEC = 'narrate commands during execution'

    # What to do by default.
    defaults = {
        FEEDBACK_PRE_CHECK_PROG: True,
        PRE_CHECK_FATAL: True,
        FEEDBACK_PRE_CHECK_EXEC: True,
        NARRATE_PROG: True,
        NARRATE_EXEC: True,
    }
