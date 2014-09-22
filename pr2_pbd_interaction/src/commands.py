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
    NO_STEPS = 'no steps were programmed before executing'


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

    @staticmethod
    def get_rr(pbdobj, rel_pos_str, arm_idx):
        '''
        Helper method for those commands that use relative (to object)
        poses.

        A relative pose will change based on whether the hand is holding
        an object. In that case, the hand's present orientation should
        be preserved. Otherwise, the robot may tip the object or orient
        it in a bad way. However, if the hand is not holding an object,
        the hand's orientation should be changed, as it should align
        with the destination object.

        Note that the above paragraph describes a heuristic, albeit a
        useful one.

        This should be called just before attempting to move, as it uses
        the current orientation of the hand.

        Args:
            pbdobj (PbdObject): The object to be relative to.
            rel_pos_str (str): The string describing how to be relative
                to pbdobj (e.g. HandsFreeCommand.[...]).
            arm_idx (int): Side.LEFT or Side.RIGHT

        Returns:
            ReachableResult
        '''
        gs = Link.get_gripper_state(arm_idx)
        if gs != GripperState.HOLDING:
            return pbdobj.reachability_map[rel_pos_str][arm_idx]
        else:
            return ObjectsHandler.get_reachable_with_cur_orient(
                pbdobj,
                rel_pos_str,
                arm_idx
            )

    @classmethod
    def get_option(cls, option_name):
        '''Gets the option that is set as a class variable.'''
        return cls.options.get(option_name)


class MoveAbsolutePose(Command):
    '''Action 1: Move the robot's gripper(s) to an absolute pose on one
    (or both?) side(s).

    self.args should have:
        [0] - side (right or left hand)
        [1] - absolute pose

    self.phrases should indicate:
        [0] - this verb (~move)
        [1] - side
        [2] - absolute pose
    '''

    options = CommandOptions({
    })

    def init(self):
        # Initialize some of our own state for convenience.
        self.arm_idx = Link.get_arm_index(self.args[0])

    def core(self):
        '''Does the movement.'''
        fb = self.default_core_feedback()
        success = Link.move_to_named_position(self.args[1], self.arm_idx)
        return success, fb


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
        self.rel_pos_str = self.args[1]

    def pre_check(self):
        '''Ensures reaching can happen.'''
        # Ensure object exists.
        # TODO(mbforbes): Re-ground object; this name is no longer valid
        # by execution time.
        pbdobj = ObjectsHandler.get_obj_by_name(self.args[2])
        if pbdobj is None:
            return False, FailureFeedback(
                'Cannot find ' + self.phrases_processed[3])

        # Ensure object is reachable.
        rr = Command.get_rr(pbdobj, self.rel_pos_str, self.arm_idx)
        if not rr.reachable:
            return False, FailureFeedback(' '.join([
                self.phrases_processed[2],
                self.phrases_processed[3],
                'is not reachable with',
                self.phrases_processed[1],
                '.'
            ]))

        # Else, sucess.
        return True, self.default_pre_feedback()

    def core(self):
        '''Does the movement.'''
        # TODO(mbforbes): Re-ground object; this name is no longer valid
        # by execution time.
        pbdobj = ObjectsHandler.get_obj_by_name(self.args[2])
        rr = Command.get_rr(pbdobj, self.rel_pos_str, self.arm_idx)
        res = Link.move_to_computed_pose(self.arm_idx, rr.pose)
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


class MoveRelativeDirection(Command):
    '''Action 4: Move the robot's gripper(s) in a relative direction to
    an object on one (or both?) side(s).

    self.args should have:
        [0] - side (right or left hand)
        [1] - relative direction (towards, away)
        [2] - object


    self.phrases should indicate:
        [0] - this verb (~move)
        [1] - side (right or left hand)
        [2] - relative direction (towards, away)
        [3] - object
    '''

    options = CommandOptions({
    })

    def init(self):
        # Initialize some of our own state for convenience.
        self.pbdobj = ObjectsHandler.get_obj_by_name(self.args[2])

    def pre_check(self):
        # Check 1: does obj exist?
        if self.pbdobj is None:
            return (
                False,
                FailureFeedback('Cannot find ' + self.phrases_processed[3]))

        # Check 2: Can we move to the requested position?
        # TODO(mbforbes): Might be useful to check if we can do this
        # without colliding with other objects...
        res = Link.get_rel_dir_possible(
            self.args[0], self.pbdobj, self.args[1])
        return res, self.default_pre_feedback()

    def core(self):
        '''Do the movement.'''
        fb = self.default_core_feedback()
        suc = Link.move_rel_dir(self.args[0], self.pbdobj, self.args[1])
        return suc, fb


class PlaceAbsoluteLocation(Command):
    '''Action 5: Place at an absolute location.

    self.args should have:
        [0] - absolute location, currently only on table / there (same)
        [1] - side (right, left hand)

    self.phrases should indicate:
        [0] - this verb (~place)
        [1] - absolute location
        [2] - side (right or left hand)
    '''

    options = CommandOptions({
    })

    def init(self):
        # Initialize some of our own state for convenience.
        self.arm_idx = Link.get_arm_index(self.args[1])
        self.narration = ' '.join([
            self.phrases_processed[0],
            self.phrases_processed[1],
            'with',
            self.phrases_processed[2]
        ])

    def pre_check(self):
        '''Ensures placing can happen.'''
        # NOTE(mbforbes): We don't have good enough 'has object'
        # detection yet because of thin objects (though haven't
        # measured). So, we just check for 'open.'
        res = True
        fb = FailureFeedback(
            "Can't " + self.phrases_processed[0] + ' as ' +
            self.phrases_processed[2] + ' is not holding an object.')
        # Must be holding to place.
        if Link.get_gripper_state(self.arm_idx) != GripperState.HOLDING:
            res = False
        return res, fb

    def narrate(self):
        '''Describes the process of placing.'''
        return Feedback(self.narration)

    def core(self):
        '''Places an object.'''
        fb = FailureFeedback(' '.join(['Failed to ' + self.narration]))

        # Move above table, but don't worry if it fails.
        Link.move_above_table(self.arm_idx)

        # Drop it.
        success = Link.set_gripper_state(self.arm_idx, GripperState.OPEN)
        return success, fb


class PlaceRelativeLocation(Command):
    '''Action 6: Place at a relative location.

    self.args should have:
        [0] - relative position
        [1] - object name
        [2] - side (right, left hand)

    self.phrases should indicate:
        [0] - this verb (~place)
        [1] - relative position
        [2] - object name
        [3] - side (right, left hand)
    '''

    options = CommandOptions({
    })

    def init(self):
        # Initialize some of our own state for convenience.
        self.rel_pos_str = self.args[0]
        self.arm_idx = Link.get_arm_index(self.args[2])
        self.narration = ' '.join([
            self.phrases_processed[0],
            self.phrases_processed[1],
            self.phrases_processed[2],
            'with',
            self.phrases_processed[3]
        ])

    def pre_check(self):
        '''Ensures reaching can happen.'''
        # Ensure object exists.
        # TODO(mbforbes): Re-ground object; this name is no longer valid
        # by execution time.
        pbdobj = ObjectsHandler.get_obj_by_name(self.args[1])
        if pbdobj is None:
            # No such object.
            return False, FailureFeedback(
                'Cannot find ' + self.phrases_processed[2])

        # Ensure object is reachable.
        rr = Command.get_rr(pbdobj, self.rel_pos_str, self.arm_idx)
        if not rr.reachable:
            # Can't reach location.
            return False, FailureFeedback(' '.join([
                self.phrases_processed[1],
                self.phrases_processed[2],
                'is not reachable with',
                self.phrases_processed[3],
                '.'
            ]))

        # Must be holding to place.
        if Link.get_gripper_state(self.arm_idx) != GripperState.HOLDING:
            # Not holding; can't place.
            return False, FailureFeedback(
                "Can't " + self.phrases_processed[0] + ' as ' +
                self.phrases_processed[2] + ' is not holding an object.')

        # Everything OK.
        return True, self.default_pre_feedback()

    def narrate(self):
        '''Describes the process of placing.'''
        return Feedback(self.narration)

    def core(self):
        '''Places an object.'''
        fb = FailureFeedback(' '.join(['Failed to ' + self.narration]))

        # Move
        # TODO(mbforbes): Re-ground object; this name is no longer valid
        # by execution time.
        pbdobj = ObjectsHandler.get_obj_by_name(self.args[1])
        rr = Command.get_rr(pbdobj, self.rel_pos_str, self.arm_idx)
        if not Link.move_to_computed_pose(self.arm_idx, rr.pose):
            return False, fb

        # Open
        success = Link.set_gripper_state(self.arm_idx, GripperState.OPEN)
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


class PointTo(Command):
    '''Action 8: Point to an object with a hand.

    self.args should have:
        [0] - object name
        [1] - side (right or left hand)

    self.phrases should indicate:
        [0] - this verb (~point to)
        [1] - object referring phrase
        [2] - side
    '''

    options = CommandOptions({
    })

    def init(self):
        # Initialize some of our own state for convenience.
        self.pbdobj = ObjectsHandler.get_obj_by_name(self.args[0])
        self.arm_idx = Link.get_arm_index(self.args[1])
        self.narration = ' '.join([
            self.phrases_processed[0],
            self.phrases_processed[1],
            'with',
            self.phrases_processed[2]
        ])

    def pre_check(self):
        '''Ensures pointing can happen.'''
        # Only check for existance of object; any pointing failure is
        # an implementation failure.
        if self.pbdobj is None:
            # No such object.
            return False, FailureFeedback(
                'Cannot find ' + self.phrases_processed[2])
        return True, self.default_pre_feedback()

    def narrate(self):
        '''Describes the process of pointing.'''
        return Feedback(self.narration)

    def core(self):
        '''Points to an object.'''
        fb = FailureFeedback(' '.join(['Failed to ' + self.narration]))
        return Link.point_to(self.pbdobj, self.arm_idx), fb


class Rotate(Command):
    '''Action 9: Rotate one of the robot's joints (currently just
    gripper roll joint).

    self.args should have:
        [0] - side (right or left hand)
        [1] - rot_dir (clockwise / counterclockwise)

    self.phrases should indicate:
        [0] - this verb (~rotate)
        [1] - side (right or left hand)
        [2] - rot_dir (clockwise / counterclockwise)
    '''

    options = CommandOptions({
    })

    def init(self):
        # Initialize some of our own state for convenience.
        self.arm_idx = Link.get_arm_index(self.args[0])

    def core(self):
        '''Rotates whichever gripper in whichever direction.'''
        res = Link.rotate(self.arm_idx, self.args[1])
        fb = self.default_core_feedback()
        return res, fb


class LookAt(Command):
    '''Action 10: Look at an object.

    self.args should have:
        [0] - object name

    self.phrases should indicate:
        [0] - this verb (~look at)
        [1] - object referring phrase
    '''

    options = CommandOptions({
    })

    def init(self):
        # Initialize some of our own state for convenience.
        self.pbdobj = ObjectsHandler.get_obj_by_name(self.args[0])

    def pre_check(self):
        '''Ensures looking at can happen.'''
        res = True
        fb = FailureFeedback('Cannot find ' + self.phrases_processed[1])
        if self.pbdobj is None:
            res = False
        return res, fb

    def core(self):
        '''Looks at the object.'''
        res = Link.look_at_object(self.pbdobj)
        fb = self.default_core_feedback()
        return res, fb


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
        # Can open if closed or holding. Cannot be open.
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
        # Cannot be closed or holding. Must be open.
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
        # Can't close if closed or holding. Must be open.
        res = Link.get_gripper_state(self.arm_idx) == GripperState.OPEN
        fb = FailureFeedback(self.phrases_processed[1] + ' is not open.')
        return res, fb

    def core(self):
        '''Closes whichever gripper.'''
        res = Link.set_gripper_state(self.arm_idx, GripperState.CLOSED)
        fb = self.default_core_feedback()
        return res, fb

    def post_check(self):
        '''Checks whether opening happened.'''
        # Success is closed or holding. Cannot be open.
        res = Link.get_gripper_state(self.arm_idx) != GripperState.OPEN
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
        # Action 1
        HandsFreeCommand.MOVE_ABSOLUTE_POSE: MoveAbsolutePose,
        # Action 2
        HandsFreeCommand.MOVE_RELATIVE_POSITION: MoveRelativePosition,
        # Action 3
        HandsFreeCommand.MOVE_ABSOLUTE_DIRECTION: MoveAbsoluteDirection,
        # Action 4
        HandsFreeCommand.MOVE_RELATIVE_DIRECTION: MoveRelativeDirection,
        # Action 5
        HandsFreeCommand.PLACE_AT_LOCATION: PlaceAbsoluteLocation,
        # Action 6
        HandsFreeCommand.PLACE_RELATIVE: PlaceRelativeLocation,
        # Action 7
        HandsFreeCommand.PICKUP: PickUp,
        # Action 8
        HandsFreeCommand.POINT_TO: PointTo,
        # Action 9
        HandsFreeCommand.ROTATE: Rotate,
        # Action 10
        HandsFreeCommand.LOOK_AT: LookAt,
        # Action 11
        HandsFreeCommand.OPEN: Open,
        # Action 12
        HandsFreeCommand.CLOSE: Close,
    }
