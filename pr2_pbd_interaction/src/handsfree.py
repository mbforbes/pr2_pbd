'''Hands-Free PbD system. Separate from PbD, living here now for
convenience.'''

# ######################################################################
# Imports
# ######################################################################

# Core ROS imports come first.
import roslib
roslib.load_manifest('pr2_pbd_interaction')
import rospy

# PbD (3rd party / local)
from arms import Arms
from pr2_pbd_interaction.msg import (
    GripperState, HandsFreeCommand, Side)
from pr2_social_gaze.msg import GazeGoal
from response import Response
from world import World

# ######################################################################
# Module level constants
# ######################################################################


# ######################################################################
# Classes
# ######################################################################

class Feedback(object):
    '''Becuase 'Response' was already taken in normal PbD.'''

    def __init__(self, speech=None, gaze=None):
        '''
        Args:
            speech (str, optional): What to say. Defaults to None.
            gaze (int, optional): Head movement to do. Should be one of
                the constants defined in GazeAction.action. Defaults to
                None.
        '''
        self.speech = speech
        self.gaze = gaze

    def issue(self):
        '''Issues the Feedback (says and/or gazes).'''
        if self.speech is not None:
            Response.say(self.speech)
        if self.gaze is not None:
            Response.perform_gaze_action(self.gaze)


class FailureFeedback(Feedback):
    '''Shakes head and says whatever.'''

    def __init__(self, speech=None):
        '''
        Args:
            speech (str, optional): What to say. Defaults to None.
        '''
        super(FailureFeedback, self).__init__(speech, GazeGoal.SHAKE)


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


class Options(object):
    '''A safe accessor to options with defaults.'''

    defaults = {}

    def __init__(self, options):
        '''
        Args:
            options (dict)
        '''
        self.options = options
        # Convenience check to avoid redundant specifications.
        for opt, val in options.iteritems():
            if self.get_default(opt) == val:
                rospy.logwarn(
                    'Specified option same as default. Option: ' + str(opt) +
                    ', Value: ' + str(val) + '.')

    def get(self, option_name):
        if option_name in self.options:
            return self.options[option_name]
        else:
            # Python awesome note: you can call class methods from an
            # instance, and it passes the class (cls) instead of the
            # instance (self). How cool is that?!?
            return self.get_default(option_name)

    @classmethod
    def get_default(cls, option_name):
        return cls.defaults[option_name]


class GlobalOptions(Options):
    '''Options for the running of the system.'''

    # What options we can specify.
    ABORT_PRECHECK = 'abort program execution on precheck failure'
    ABORT_CORE = 'abort program execution on command execution failure'
    ABORT_POSTCHECK = 'abort program execution on postcheck failure'

    # TODO(mbforbes): Load from ROS params!
    defaults = {
        ABORT_PRECHECK: True,
        ABORT_CORE: True,
        ABORT_POSTCHECK: True,
    }


class CommandOptions(Options):
    '''How to execute a Command.'''

    # What options we can specify.
    FEEDBACK_PRECHECK_PROG = (
        'notify user when precheck fails during programming')
    PRECHECK_FATAL = (
        'whether the execution should normally be stopped if pre-check fails')
    FEEDBACK_PRECHECK_EXEC = 'notify user when precheck fails during execution'
    NARRATE_PROG = 'narrate commands during programming'
    NARRATE_EXEC = 'narrate commands during execution'

    # What to do by default.
    defaults = {
        FEEDBACK_PRECHECK_PROG: True,
        PRECHECK_FATAL: True,
        FEEDBACK_PRECHECK_EXEC: True,
        NARRATE_PROG: True,
        NARRATE_EXEC: False,
    }


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

        # Initialize any command-specific code.
        if hasattr(self, 'init'):
            self.init()

    def execute(self, mode):
        '''Executes this command in mode.

        Args:
            mode (str): One of Mode.*

        Returns:
            str: One of Code.*.
        '''
        # Do pre-check.
        if hasattr(self, 'pre_check'):
            pre_success, pre_feedback = self.pre_check(self.args, self.phrases)
            if not pre_success:
                # Maybe issue feedback for this.
                if ((mode == Mode.PROG and
                        self.get_option(
                            CommandOptions.FEEDBACK_PRECHECK_PROG)) or
                    (mode == Mode.EXEC and
                        self.get_option(
                            CommandOptions.FEEDBACK_PRECHECK_EXEC))):
                    pre_feedback.issue()
                if self.get_option(CommandOptions.PRE_CHECK_FATAL):
                    return Code.PRE_CHECK_FAIL
                else:
                    return Code.PRE_CHECK_NO_OP

        # About to do the action; narrate if necessary.
        if hasattr(self, 'narrate'):
            if ((mode == Mode.EXEC and
                    self.get_option(CommandOptions.NARRATE_EXEC)) or
                    mode == Mode.PROG and
                    self.get_option(CommandOptions.NARRATE_PROG)):
                narrate_feedback = self.narrate(self.args, self.phrases)
                narrate_feedback.issue()

        # Do the action.
        if hasattr(self, 'core'):
            exec_success, exec_feedback = self.core(self.args, self.phrases)
            if not exec_success:
                exec_feedback.issue()
                return Code.EXEC_FAIL

        # Do post-check.
        if hasattr(self, 'post_check'):
            post_success, post_feedback = self.post_check(
                self.args, self.phrases)
            if not post_success:
                post_feedback.issue()
                return Code.POST_CHECK_FAIL

        # We succeeded!
        return Code.SUCCESS

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
    '''

    options = CommandOptions({
    })

    # def init(self):
    #     # Initialize some of our own state for convenience.
    #     self.arm_idx = Link.get_arm_index(args[0])
    #     # TODO(mbforbes): Should use phrases.
    #     self.hand_str = 'right' if self.arm_idx == Side.RIGHT else 'left'

    # def pre_check(self, args, phrases):
    #     '''Ensures closing can happen.'''
    #     res = S.arms.get_gripper_state(self.arm_idx) != GripperState.CLOSED
    #     fb = FailureFeedback(self.hand_str + ' is already closed.')
    #     return res, fb

    # def narrate(self, args, phrases):
    #     '''Describes the process of closing.'''
    #     fb = Feedback('Closing ' + self.hand_str + ' hand.')
    #     return fb

    # def core(self, args, phrases):
    #     '''Closes whichever gripper.'''
    #     res = S.arms.set_gripper_state(self.arm_idx, GripperState.CLOSED)
    #     fb = FailureFeedback(self.hand_str + ' failed to close.')
    #     return res, fb

    # def post_check(self, args, phrases):
    #     '''Checks whether opening happened.'''
    #     res = S.arms.get_gripper_state(self.arm_idx) == GripperState.CLOSED
    #     fb = FailureFeedback(self.hand_str + ' did not close.')
    #     return res, fb


class Open(Command):
    '''Action 11: Open the robot's gripper on one (or both?) side(s).

    self.args should have:
        [0] - side (right or left hand)
    '''

    options = CommandOptions({
        CommandOptions.FEEDBACK_PRECHECK_EXEC: False,
        CommandOptions.PRECHECK_FATAL: False,
    })

    def init(self):
        # Initialize some of our own state for convenience.
        self.arm_idx = Link.get_arm_index(args[0])
        # TODO(mbforbes): Should use phrases.
        self.hand_str = 'right' if self.arm_idx == Side.RIGHT else 'left'

    def pre_check(self, args, phrases):
        '''Ensures opening can happen.'''
        res = S.arms.get_gripper_state(self.arm_idx) != GripperState.OPEN
        fb = FailureFeedback(self.hand_str + ' is already open.')
        return res, fb

    def narrate(self, args, phrases):
        '''Describes the process of opening.'''
        fb = Feedback('Opening ' + self.hand_str + ' hand.')
        return fb

    def core(self, args, phrases):
        '''Opens whichever gripper.'''
        res = S.arms.set_gripper_state(self.arm_idx, GripperState.OPEN)
        fb = FailureFeedback(self.hand_str + ' failed to open.')
        return res, fb

    def post_check(self, args, phrases):
        '''Checks whether opening happened.'''
        res = S.arms.get_gripper_state(self.arm_idx) == GripperState.OPEN
        fb = FailureFeedback(self.hand_str + ' did not open.')
        return res, fb


class Close(Command):
    '''Action 12: Close the robot's gripper on one (or both?) side(s).

    self.args should have:
        [0] - side (right or left hand)
    '''

    options = CommandOptions({
        CommandOptions.FEEDBACK_PRECHECK_EXEC: False,
        CommandOptions.PRECHECK_FATAL: False,
    })

    def init(self):
        # Initialize some of our own state for convenience.
        self.arm_idx = Link.get_arm_index(args[0])
        # TODO(mbforbes): Should use phrases.
        self.hand_str = 'right' if self.arm_idx == Side.RIGHT else 'left'

    def pre_check(self, args, phrases):
        '''Ensures closing can happen.'''
        res = S.arms.get_gripper_state(self.arm_idx) != GripperState.CLOSED
        fb = FailureFeedback(self.hand_str + ' is already closed.')
        return res, fb

    def narrate(self, args, phrases):
        '''Describes the process of closing.'''
        fb = Feedback('Closing ' + self.hand_str + ' hand.')
        return fb

    def core(self, args, phrases):
        '''Closes whichever gripper.'''
        res = S.arms.set_gripper_state(self.arm_idx, GripperState.CLOSED)
        fb = FailureFeedback(self.hand_str + ' failed to close.')
        return res, fb

    def post_check(self, args, phrases):
        '''Checks whether opening happened.'''
        res = S.arms.get_gripper_state(self.arm_idx) == GripperState.CLOSED
        fb = FailureFeedback(self.hand_str + ' did not close.')
        return res, fb


class Program(object):
    '''Holds what the user programs (a list of Commands).'''

    def __init__(self, options):
        '''
        Args:
            options (GlobalOptions)
        '''
        self.commands = []
        self.options = options

    def execute(self):
        '''
        Executes the commands that have been programmed.
        '''
        self.running = True
        for command in self.commands:
            # Ensure we haven't been stopped.
            if not self.running:
                break

            # Execute the command.
            code = command.execute(Mode.EXEC)

            # Break if code bad and set to abort on that bad code.
            if ((code == Code.PRE_CHECK_FAIL and
                    self.options.get(GlobalOptions.ABORT_PRECHECK)) or
                    (code == Code.EXEC_FAIL and
                    self.options.get(GlobalOptions.ABORT_CORE)) or
                    (code == Code.POST_CHECK_FAIL and
                    self.options.get(GlobalOptions.ABORT_POSTCHECK))):
                break
        self.running = False

    def add_command(self, command):
        '''
        Args:
            command (Command)
        '''
        self.commands += [command]

    def is_executing(self):
        '''
        Returns whether the program is currently executing.

        Returns:
            bool
        '''
        return self.running

    def stop(self):
        '''
        Stops executing, if it is.
        '''
        self.running = False

class S:
    '''"Singletons" or "State" or whatever you want to call it.

    This class is to hold the global vars that commands need to access.
    The fields are truely singletons, so we might as well treat them as
    so rather than passing them around everywhere.

    The name is so short because it's easier that way.
    '''
    arms = None
    world = None


class Link:
    '''This class provides is the interface between HandsFree and the
    rest of the PbD system. It wraps variables and converts types.

    Replacing it and S would make the system PbD-independent.
    '''

    @staticmethod
    def get_arm_index(arm_str):
        '''
        Args:
            arm_str (str): HandsFreeCommand.LEFT_HAND or
                HandsFreeCommand.RIGHT_HAND

        Returns
            int: Side.RIGHT or Side.LEFT
        '''
        return (
            Side.RIGHT if arm_str == HandsFreeCommand.RIGHT_HAND else
            Side.LEFT)

class HandsFree(object):
    '''Sets up the hands-free system.'''

    # "Admin" commands (change state of system somehow).
    admin_commands = {
        HandsFreeCommand.EXECUTE: self.run_program,
        HandsFreeCommand.STOP: self.stop_program,
    }

    # "Emergency" commands list a subset of "Admin" commands that can be
    # run even while the robot is executing.
    emergency_commands = [
        HandsFreeCommand.STOP,
    ]

    # "Normal" commands (make robot do something).
    command_map = {
        HandsFreeCommand.OPEN: Open,
        HandsFreeCommand.CLOSE: Close,
    }

    def __init__(self, arms, world):
        # Save singletons.
        if S.arms is None:
            S.arms = arms
        if S.world is None:
            S.world = world

        # Set up state.
        # Eventually will have multiple programs.
        self.program = Program(GlobalOptions())

        # Set up the command dispatch.
        rospy.Subscriber(
            'handsfree_command', HandsFreeCommand, self.command_cb)

    def command_cb(self, hf_cmd):
        '''
        Callback for when praser sends us a command.

        Args:
            hf_cmd (HandsFreeCommand)
        '''
        # Extract vars for ease of use
        cmd, args, phrases = hf_cmd.cmd, hf_cmd.args, hf_cmd.phrases

        # First, check whether executing for emergency commands.
        if self.get_program().is_executing():
            if cmd not in emergency_commands:
                return

        # Check if admin or normal command.
        if cmd in admin_commands:
            # Admin: run as function.
            admin_commands[cmd]()
        else:
            # Normal: instantiate a new Command with this data.
            command = command_map[cmd](hf_cmd.args, hf_cmd.phrases)

            # Execute on the robot
            code = command.execute(Mode.PROG)

            # If it worked, we add it to the program.
            if code == Code.SUCCESS:
                self.get_program().add_command(command)

        # Always update any state changes for the parser.
        self.broadcast_state()

    def broadcast_state(self):
        '''
        Broadcasts world and robot state (to the parser).
        '''
        pass

    def get_program(self):
        '''Come on, get with the program!

        Returns:
            Program: The current program.
        '''
        return self.program

    # 'Admin' actions just trigger functions here.

    def run_program(self):
        '''
        Runs the current program.
        '''
        self.get_program.execute()

    def stop_program(self):
        '''
        Stops the current program.
        '''
        self.get_program.stop()
