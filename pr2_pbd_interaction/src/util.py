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

# Builtins
import os
from datetime import datetime

# ROS builtins
import rosbag
from rospy.exceptions import ROSException
from sensor_msgs.msg import Image
from std_msgs.msg import String

# PbD (3rd party / local)
from pr2_pbd_interaction.msg import (
    HandsFreeCommand, Description, HandsFreeGrounding)

# ######################################################################
# Module level constants
# ######################################################################

EXP_DIR = '/home/mbforbes/repos/hfpbd-data/'

# For listening and logging to ROS bag.
IMG_TOPIC = '/head_mount_kinect/rgb/image_color'

# For logging to ROS bag only.
DESC_TOPIC = '/handsfree_description'
CMD_TOPIC = '/handsfree_command'
GROUNDING_TOPIC = '/handsfree_grounding'
FB_TOPIC = '/feedback'

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


class LoggerImplementation(object):
    '''
    Helper class for recording experiment data.

    Attrs:
        self.fh (filehandle)
        self.bag (rosbag)
        self.fnum (str)
    '''

    def __init__(self):
        # Find dir
        now = datetime.now()
        yearmonth = now.strftime('%y-%m%b').lower()
        day = now.strftime('%d%a').lower()
        dir_ = EXP_DIR + yearmonth + '/' + day + '/'
        if not os.path.exists(dir_):
            os.makedirs(dir_)

        # Find empty number
        num = 1
        t_ext = '.txt'
        while os.path.exists(dir_ + str(num) + t_ext):
            num += 1

        # Open text file
        self.fnum = str(num)
        t_filename = dir_ + self.fnum + t_ext
        self.fh = open(t_filename, 'w')
        self._p(now)

        # Open bag file
        b_ext = '.bag'
        b_filename = dir_ + self.fnum + b_ext
        self.bag = rosbag.Bag(b_filename, 'w')

    def cleanup(self):
        '''
        Called before exiting.
        '''
        # I'm sure these already flush, but why not.
        self.fh.flush()
        self.fh.close()
        self.bag.flush()
        self.bag.close()

    def _p(self, obj):
        '''
        Prints obj to text log.

        Args:
            obj (object)
        '''
        self.fh.write(str(obj) + '\n')
        # For safety as ROS crashes a lot and we're not logging much.
        self.fh.flush()

    def enter_action(self):
        '''
        For visual convenience, save 'different action' separators in
        log (either 'new' or 'switch_to').
        '''
        self._p('-' * 70)

    def save_fb(self, msg):
        '''
        Logs feedback message msg and a picture to go with it.

        Args:
            msg (str)
        '''
        # Save picture
        self.save_picture()

        # Save to bag and text log.
        self.bag.write(FB_TOPIC, String(msg))
        self._p('Feedback:')
        self._p('\t' + msg)

    def save_grounding(self, hf_grounding):
        '''
        Logs grounding result and a picture to go with it.

        Args:
            hf_grounding (HandsFreeGrounding)
        '''
        # Save picture
        self.save_picture()

        # Save grounding to bag and text log.
        self.bag.write(GROUNDING_TOPIC, hf_grounding)
        obj_names, obj_probs, query = (
            hf_grounding.obj_names, hf_grounding.obj_probs, hf_grounding.query)
        self._p('Grounding:')
        self._p('\tquery: ' + query)
        for i in range(len(obj_names)):
            self._p('\t' + obj_names[i] + ': ' + str(obj_probs[i]))

    def save_cmd(self, hf_cmd):
        '''
        Saves a hands-free command and a picture to go with it.

        Args:
            hf_cmd (HandsFreeCommand)
        '''
        # Save picture first in case it is time-sensitive (e.g. robot
        # will be moving shortly after).
        self.save_picture()

        # Log command to bag and text log.
        self.bag.write(CMD_TOPIC, hf_cmd)
        cmd, args, phrases, utterance = (
            hf_cmd.cmd, hf_cmd.args, hf_cmd.phrases, hf_cmd.utterance)
        self._p('Command:')
        self._p('\t  command: ' + cmd + ': ' + ' '.join(args))
        self._p('\t  phrases: ' + ' '.join(phrases))
        self._p('\tutterance: ' + utterance)

    def save_desc(self, desc):
        '''
        Saves a description and picture to go with it.

        Args:
            desc (Description)
        '''
        # Save picture first in case it is time-sensitive (e.g. robot
        # will be moving shortly after).
        self.save_picture()

        # Log description to bag and text log.
        self.bag.write(DESC_TOPIC, desc)
        names, descs = desc.object_names, desc.descriptions
        self._p('Description:')
        for i in range(len(names)):
            self._p('\t' + names[i] + ':' + descs[i])

    def save_picture(self):
        '''
        Captures images from head kinect and saves to ros bag.
        '''
        rospy.loginfo('Trying to save picture')
        # Note that this doesn't work in simulation because of the
        # different topic. Could do conditional here, but we really only
        # care about real robot, so just doing timeout for now.
        try:
            msg = rospy.wait_for_message(IMG_TOPIC, Image, 1.0)
            self.bag.write(IMG_TOPIC, msg)
            rospy.loginfo('Done saving picture')
        except ROSException:
            rospy.loginfo("Couldn't get picture to save.")


class Logger(object):
    '''Singleton for providing access to the logger.'''
    L = LoggerImplementation()
