'''Program for Hands-Free PbD system. Separate from PbD, living here now
for convenience.'''

__author__ = 'mbforbes'


# ######################################################################
# Imports
# ######################################################################

# Core ROS imports come first.
import roslib
roslib.load_manifest('pr2_pbd_interaction')
import rospy

from commands import Mode, Code
from feedback import Feedback, FailureFeedback
from objects import ObjectsHandler
from robot import RobotHandler
from util import GlobalOptions

# ######################################################################
# Classes
# ######################################################################


class Program(object):
    '''Holds what the user programs (a list of Commands).'''

    _executing = False

    def __init__(self, options, idx_name):
        '''
        Args:
            options (GlobalOptions)
            idx_name (int): 1-based index of this program, purely for
                speech.
        '''
        # Setup state
        self.commands = []
        self.options = options
        self.idx_name = idx_name  # 1-based
        self.stop_requested = False

        # Record and broadcast objects.
        Feedback("Created action %d. Finding objects." % (idx_name)).issue()
        ObjectsHandler.record()

    def execute(self):
        '''
        Executes the commands that have been programmed.
        '''
        Feedback("Executing action %d." % (self.idx_name)).issue()
        self.set_executing(True)

        # TODO(mbforbes): Recording objects should be a step.
        # TODO(mbforbes): Step pre-checks should ensure given objects
        # exist.
        # Look for objects
        ObjectsHandler.record()

        # Execute steps in order.
        for idx, command in enumerate(self.commands):
            idx_1based = idx + 1  # For display
            cmd_name = type(command).__name__

            # Ensure we haven't been stopped.
            if self.stop_requested:
                self.stop_requested = False
                break

            # Execute the command.
            rospy.loginfo("Executing step %d: %s" % (idx_1based, cmd_name))
            code = command.execute(Mode.EXEC)

            # Break if code bad and set to abort on that bad code.
            if ((code == Code.PRE_CHECK_FAIL and
                    self.options.get(GlobalOptions.ABORT_PRE_CHECK)) or
                    (code == Code.EXEC_FAIL and
                        self.options.get(GlobalOptions.ABORT_CORE)) or
                    (code == Code.POST_CHECK_FAIL and
                        self.options.get(GlobalOptions.ABORT_POST_CHECK))):
                rospy.loginfo("Got code %s; stopping." % (code))
                break

        self.set_executing(False)

        if code == Code.SUCCESS:
            Feedback("Completed action %d." % (self.idx_name)).issue()
        else:
            FailureFeedback(
                "Problem executing action %d." % (self.idx_name)
            ).issue()

    @staticmethod
    def is_executing():
        '''
        Returns whether the program is currently executing.

        Returns:
            bool
        '''
        return Program._executing

    @staticmethod
    def set_executing(val):
        '''
        A wrapper for the internal variable so we can broadcast state
        changes.

        Args:
            val (bool):
        '''
        Program._executing = val
        RobotHandler.is_executing = val
        RobotHandler.async_broadcast()

    def switch_to(self, idx_name):
        '''
        Called when this program is switched to.

        Args:
            idx_name (int): 1-based index of this program, purely for
                speech.
        '''
        Feedback(
            'Switched to action %d. Finding objects.' % (self.idx_name)
        ).issue()
        ObjectsHandler.record()

    def add_command(self, command):
        '''
        Args:
            command (Command)
        '''
        self.commands += [command]

    def stop(self):
        '''
        Stops executing, if it is.

        This is 'lazy' in that this really just requests the progrram to
        stop before executing the next step. For true, robot-apocalypse-
        scale problems, hit the run-stop.
        '''

        self.stop_requested = True