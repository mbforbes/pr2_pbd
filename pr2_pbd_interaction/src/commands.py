'''Commands for Hands-Free PbD system. Separate from PbD, living here
now for convenience.'''

__author__ = 'mbforbes'


# ######################################################################
# Imports
# ######################################################################

# Core ROS imports come first.
import roslib
roslib.load_manifest('pr2_pbd_interaction')
import rospy

# PbD (3rd party / local)
from pr2_pbd_interaction.msg import HandsFreeCommand, Side, GripperState

# Local
from feedback import Feedback, FailureFeedback
from objects import ObjectsHandler
from robot import RobotHandler
from robotlink import Link
from util import CommandOptions


# ######################################################################
# Classes
# ######################################################################

class Mode(object):
    '''Modes in which a command can be executed.'''

    PROG = 'programming'
    EXEC = 'executing'


class Code(object):
    '''Status codes that execution functions can return.'''

    PRE_CHECK_FAIL = 'pre check failure'
    PRE_CHECK_NO_OP = 'pre check failure, but command says that is OK'
    EXEC_FAIL = 'failure during attempted execution'
    POST_CHECK_FAIL = 'post check failure'
    SUCCESS = 'successfully exected step'


class Command(object):
    '''A command holds the execution code for a parameterizable verb.

    Verbs that are truly implemented subclass Command and provide the
    following functions:
        - pre_check   (fn: args, phrases -> (bool, Feedback))
        - narrate     (fn: args, phrases -> Feedback)
        - core        (fn: args, phrases -> bool, Feedback)
        - post_check  (fn: args, phrases -> bool, Feedback)
    '''

    def __init__(self, args, phrases):
        '''
        Args:
            args ([str]): Arguments to the command.
            phrases: ([[str]]): Words matched with each arg.
        '''
        self.args = args
        self.phrases = phrases
        self.phrases_processed = Command.process_phrases(phrases)
        self.phrases_processed_str = (
            ' '.join(self.phrases_processed).capitalize() + '.')

        # Initialize any command-specific code.
        if hasattr(self, 'init'):
            self.init()

    @staticmethod
    def process_phrases(phrases):
        '''
        Removes speech-recognition-helping hyphens from phrases.

        Args:
            phrases ([str])

        Returns:
            [str]
        '''
        return [p.replace('-', ' ') for p in phrases]

    def execute(self, mode):
        '''Executes this command in mode.

        Args:
            mode (str): One of Mode.*

        Returns:
            str: One of Code.*
        '''
        # Update the last-commanded side. It is an interesting choice
        # where to put this (should it only be updated after the
        # pre-check passes? or the command succeeds?). I think as soon
        # as someone referrs to the command it should be updated, so
        # it's going here for now.
        self.update_last_side()

        # Do pre-check.
        if hasattr(self, 'pre_check'):
            pre_success, pre_feedback = self.pre_check()
            if not pre_success:
                # Maybe issue feedback for this.
                if ((mode == Mode.PROG and
                        self.get_option(
                            CommandOptions.FEEDBACK_PRE_CHECK_PROG)) or
                    (mode == Mode.EXEC and
                        self.get_option(
                            CommandOptions.FEEDBACK_PRE_CHECK_EXEC))):
                    pre_feedback.issue()
                if self.get_option(CommandOptions.PRE_CHECK_FATAL):
                    return Code.PRE_CHECK_FAIL
                else:
                    return Code.PRE_CHECK_NO_OP

        # About to do the action; narrate if necessary.
        # NOTE(mbforbes): We now have a default narrate function, so
        # this hasattr(...) check will always return true.
        if hasattr(self, 'narrate'):
            if ((mode == Mode.EXEC and
                    self.get_option(CommandOptions.NARRATE_EXEC)) or
                    mode == Mode.PROG and
                    self.get_option(CommandOptions.NARRATE_PROG)):
                narrate_feedback = self.narrate()
                narrate_feedback.issue()

        # Do the action.
        if hasattr(self, 'core'):
            exec_success, exec_feedback = self.core()
            if not exec_success:
                exec_feedback.issue()
                return Code.EXEC_FAIL

        # Do post-check.
        if hasattr(self, 'post_check'):
            post_success, post_feedback = self.post_check()
            if not post_success:
                post_feedback.issue()
                return Code.POST_CHECK_FAIL

        # We succeeded!
        return Code.SUCCESS

    def update_last_side(self):
        '''
        Updates the last-commanded side.
        '''
        # This is kind of a hack, because we're assuming command
        # arguments don't overlap. But we get to choose how command
        # arguments are represented internally, so this isn't so bad.
        check_sides = [
            # Note that these should match RobotState.* relevant consts.
            HandsFreeCommand.RIGHT_HAND,
            HandsFreeCommand.LEFT_HAND,
            # NOTE(mbforbes); Eventually do both hands...
        ]
        for arg in self.args:
            for side in check_sides:
                if arg == side:
                    RobotHandler.last_commanded = side
                    return

    def narrate(self):
        '''
        Default narration; uses processed phrases.

        Returns:
            Feedback
        '''
        # Got to remove hyphens and underscores for speaking.
        return Feedback(self.phrases_processed_str)

    # Helpers
    def default_pre_feedback(self):
        '''
        Default feedback for a failed pre-check.

        NOTE(mbforbes): This is not used automatically, as you still
        must implement an actual pre_check function. However, this may
        be used in pre_check functions as an easy default response.

        Returns: FailureFeedback
        '''
        return FailureFeedback('Cannot ' + self.phrases_processed_str)

    def default_core_feedback(self):
        '''
        Default feedback for a failed core (actual execution).

        NOTE(mbforbes): This is not used automatically, as you still
        must implement an core function. However, this may be used in
        core functions as an easy default response.

        Returns: FailureFeedback
        '''
        return FailureFeedback('Failed to ' + self.phrases_processed_str)

    @classmethod
    def get_option(cls, option_name):
        '''Gets the option that is set as a class variable.'''
        return cls.options.get(option_name)


class MoveRelativePosition(Command):
    '''Action 2: Move the robot's gripper(s) to a relative position on
    one (or both?) side(s).

    self.args should have:
        [0] - side (right or left hand)
        [1] - relative position
        [2] - object (to move relative to)

    self.phrases should indicate:
        [0] - this verb (~move)
        [1] - side
        [2] - relative position
        [3] - object (to move relative to)
    '''

    options = CommandOptions({
    })

    def init(self):
        # Initialize some of our own state for convenience.
        self.arm_idx = Link.get_arm_index(self.args[0])
        self.pbdobj = ObjectsHandler.get_obj_by_name(self.args[2])
        if self.pbdobj is not None:
            self.rr = self.pbdobj.reachability_map[self.args[1]][self.arm_idx]

    def pre_check(self):
        '''Ensures reaching can happen.'''
        if self.pbdobj is None:
            return False, FailureFeedback(
                'Cannot find ' + self.phrases_processed[3])
        elif not self.rr.reachable:
            return False, FailureFeedback(' '.join([
                self.phrases_processed[2],
                self.phrases_processed[3],
                'is not reachable.']))
        else:
            return True, self.default_pre_feedback()

    def core(self):
        '''Does the movement.'''
        res = Link.move_to_computed_pose(self.arm_idx, self.rr.pose)
        fb = self.default_core_feedback()
        return res, fb

    # TODO(mbforbes): Post-check that relative move worked.


class MoveAbsoluteDirection(Command):
    '''Action 3: Move the robot's gripper(s) in an absolute direction on
    one (or both?) side(s).

    self.args should have:
        [0] - side (right or left hand)
        [1] - absolute direction

    self.phrases should indicate:
        [0] - this verb (~move)
        [1] - side
        [2] - absolute direction
    '''

    options = CommandOptions({
    })

    def init(self):
        # Initialize some of our own state for convenience.
        self.arm_idx = Link.get_arm_index(self.args[0])

    def pre_check(self):
        '''Ensures moving in the specified direction can happen.'''
        res = Link.get_abs_dir_possible(self.args[0], self.args[1])
        fb = self.default_pre_feedback()
        return res, fb

    def core(self):
        '''Moves.'''
        success = Link.move_abs_dir(self.args[0], self.args[1])
        fb = self.default_core_feedback()
        return success, fb


class PickUp(Command):
    '''Action 7: Pick up an object with one (or both?) hand(s).

    self.args should have:
        [0] - object name
        [1] - side (right or left hand)

    self.phrases should indicate:
        [0] - this verb (~pick up)
        [1] - object referring phrase
        [2] - side
    '''

    options = CommandOptions({
    })

    def init(self):
        # Initialize some of our own state for convenience.
        self.arm_idx = Link.get_arm_index(self.args[1])
        self.obj_str = self.args[0]
        self.narration = ' '.join([
            self.phrases_processed[0],
            self.phrases_processed[1],
            'with',
            self.phrases_processed[2]
        ])

    def pre_check(self):
        '''Ensures picking up can happen.'''
        # NOTE(mbforbes): Not sure of a way to conveniently check this
        # with the current implementation. That's not to say it isn't
        # possible.
        pbdobj = ObjectsHandler.get_obj_by_name(self.args[0])
        res = True
        # This would be the 'default' failure mode (but we never do a
        # real check so it never happens.)
        fb = FailureFeedback(' '.join(['Cannot', self.narration]))

        if pbdobj is None:
            res = False
            fb = FailureFeedback('Cannot find ' + self.phrases_processed[1])
        return res, fb

    def narrate(self):
        '''Describes the process of picking up.'''
        return Feedback(self.narration)

    def core(self):
        '''Picks up object.'''
        pbdobj = ObjectsHandler.get_obj_by_name(self.args[0])
        success = False
        fb = FailureFeedback(' '.join(['Failed to ' + self.narration]))
        if pbdobj is not None:
            success = Link.pick_up(pbdobj, self.arm_idx)
        return success, fb


class Open(Command):
    '''Action 11: Open the robot's gripper on one (or both?) side(s).

    self.args should have:
        [0] - side (right or left hand)

    self.phrases should indicate:
        [0] - this verb (~open)
        [1] - side
    '''

    options = CommandOptions({
        CommandOptions.FEEDBACK_PRE_CHECK_EXEC: False,
        CommandOptions.PRE_CHECK_FATAL: False,
    })

    def init(self):
        # Initialize some of our own state for convenience.
        self.arm_idx = Link.get_arm_index(self.args[0])

    def pre_check(self):
        '''Ensures opening can happen.'''
        res = Link.get_gripper_state(self.arm_idx) != GripperState.OPEN
        fb = FailureFeedback(self.phrases_processed[1] + ' is already open.')
        return res, fb

    def core(self):
        '''Opens whichever gripper.'''
        res = Link.set_gripper_state(self.arm_idx, GripperState.OPEN)
        fb = self.default_core_feedback()
        return res, fb

    def post_check(self):
        '''Checks whether opening happened.'''
        res = Link.get_gripper_state(self.arm_idx) == GripperState.OPEN
        fb = FailureFeedback(' '.join([
            self.phrases_processed[1],
            'did not',
            self.phrases_processed[0]
        ]))
        return res, fb


class Close(Command):
    '''Action 12: Close the robot's gripper on one (or both?) side(s).

    self.args should have:
        [0] - side (right or left hand)

    self.phrases should indicate:
        [0] - this verb (~close)
        [1] - side
    '''

    options = CommandOptions({
        CommandOptions.FEEDBACK_PRE_CHECK_EXEC: False,
        CommandOptions.PRE_CHECK_FATAL: False,
    })

    def init(self):
        # Initialize some of our own state for convenience.
        self.arm_idx = Link.get_arm_index(self.args[0])

    def pre_check(self):
        '''Ensures closing can happen.'''
        res = Link.get_gripper_state(self.arm_idx) != GripperState.CLOSED
        fb = FailureFeedback(self.phrases_processed[1] + ' is already closed.')
        return res, fb

    def core(self):
        '''Closes whichever gripper.'''
        res = Link.set_gripper_state(self.arm_idx, GripperState.CLOSED)
        fb = self.default_core_feedback()
        return res, fb

    def post_check(self):
        '''Checks whether opening happened.'''
        res = Link.get_gripper_state(self.arm_idx) == GripperState.CLOSED
        fb = FailureFeedback(' '.join([
            self.phrases_processed[1],
            'did not',
            self.phrases_processed[0]
        ]))
        return res, fb


class CommandRouter(object):
    '''Maps commands to classes and functions.'''

    # Set up admin callbacks (change state of system somehow).
    admin_commands = {
        HandsFreeCommand.EXECUTE: 'execute_program',
        HandsFreeCommand.STOP: 'stop_program',
        HandsFreeCommand.CREATE_NEW_ACTION: 'create_new_action',
        HandsFreeCommand.SWITCH_ACTION: 'switch_to_action',
        HandsFreeCommand.CLARIFY: 'clarify',
    }

    # "Emergency" commands list a subset of "Admin" commands that can be
    # run even while the robot is executing.
    emergency_commands = [
        HandsFreeCommand.STOP,
    ]

    # "Normal" commands (make robot do something).
    command_map = {
        HandsFreeCommand.MOVE_ABSOLUTE_DIRECTION: MoveAbsoluteDirection,
        HandsFreeCommand.MOVE_RELATIVE_POSITION: MoveRelativePosition,
        HandsFreeCommand.PICKUP: PickUp,
        HandsFreeCommand.OPEN: Open,
        HandsFreeCommand.CLOSE: Close,
    }
