'''Hands-Free PbD system. Separate from PbD, living here now for
convenience.'''

# ######################################################################
# Imports
# ######################################################################

# Core ROS imports come first.
import roslib
roslib.load_manifest('pr2_pbd_interaction')
import rospy

# Builtins
from threading import Thread

# PbD (3rd party / local)
from arms import Arms
from pr2_pbd_interaction.msg import (
    GripperState, HandsFreeCommand, WorldObjects, WorldObject, RobotState,
    Side)
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

    NO_PROGRAM = FailureFeedback('No actions to execute.')
    NO_PROGRAM_SWITCH = FailureFeedback('No actions created to switch to.')
    NOT_EXECUTING = FailureFeedback('Not executing action; cannot stop.')
    NO_PREVIOUS = FailureFeedback('No previous action.')
    NO_NEXT = FailureFeedback('No next action.')

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

    def __init__(self, options={}):
        '''
        Args:
            options (dict, optional): Defaults to {}.
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
        CommandOptions.FEEDBACK_PRE_CHECK_EXEC: False,
        CommandOptions.PRE_CHECK_FATAL: False,
    })

    def init(self):
        # Initialize some of our own state for convenience.
        self.arm_idx = Link.get_arm_index(self.args[0])
        # TODO(mbforbes): Should use phrases.
        self.hand_str = (
            ('right' if self.arm_idx == Side.RIGHT else 'left') + ' hand')

    def pre_check(self, args, phrases):
        '''Ensures opening can happen.'''
        res = S.arms.get_gripper_state(self.arm_idx) != GripperState.OPEN
        fb = FailureFeedback(self.hand_str + ' is already open.')
        return res, fb

    def narrate(self, args, phrases):
        '''Describes the process of opening.'''
        fb = Feedback('Opening ' + self.hand_str + '.')
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
        CommandOptions.FEEDBACK_PRE_CHECK_EXEC: False,
        CommandOptions.PRE_CHECK_FATAL: False,
    })

    def init(self):
        # Initialize some of our own state for convenience.
        self.arm_idx = Link.get_arm_index(self.args[0])
        # TODO(mbforbes): Should use phrases.
        self.hand_str = (
            ('right' if self.arm_idx == Side.RIGHT else 'left') + ' hand')

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

    world_object_pub = rospy.Publisher('handsfree_worldobjects', WorldObjects)

    def __init__(self, options):
        '''
        Args:
            options (GlobalOptions)
        '''
        # Setup state
        self.commands = []
        self.options = options
        self.running = False
        self.world_objects = []

        # Record and broadcast objects.
        self.record_world_objects()

    def execute(self):
        '''
        Executes the commands that have been programmed.
        '''
        # TODO(mbforbes): Say something when you start.

        self.running = True
        for command in self.commands:
            # Ensure we haven't been stopped.
            if not self.running:
                break

            # Execute the command.
            code = command.execute(Mode.EXEC)

            # Break if code bad and set to abort on that bad code.
            if ((code == Code.PRE_CHECK_FAIL and
                    self.options.get(GlobalOptions.ABORT_PRE_CHECK)) or
                    (code == Code.EXEC_FAIL and
                        self.options.get(GlobalOptions.ABORT_CORE)) or
                    (code == Code.POST_CHECK_FAIL and
                        self.options.get(GlobalOptions.ABORT_POST_CHECK))):
                break
        self.running = False

        # TODO(mbforbes): Say something when you finish (depending on
        # code).

    def switch_to(self, idx_name):
        '''
        Called when this program is switched to.

        Args:
            idx_name (int): 1-based index of this program, purely for
                speech.
        '''
        Feedback(
            'Switched to action %d. Finding objects.' % (idx_name)).issue()
        self.record_world_objects()

    def add_command(self, command):
        '''
        Args:
            command (Command)
        '''
        self.commands += [command]

    def record_world_objects(self):
        '''
        Records and broadcasts world objects.
        '''
        self._record_world_objects_internal()
        self._broadcast()

    def update_world_objects(self):
        '''
        Updates the existing world objects by assuming they haven't
        changed and computing reachabilities. Also broadcasts.
        '''
        # TODO(mbforbes): Implement.
        # - compute properties of existing objects
        self._update_world_objects_internal()
        self._broadcast()

    def _record_world_objects_internal(self):
        '''
        Gets the world objects (actually observes).
        '''
        # TODO(mbforbes): Implement.
        # - look down, record
        self.world_objects = WorldObjects()
        self._update_world_objects_internal()

    def _update_world_objects_internal(self):
        '''
        Updates reachability properties of existing WorldObjects.
        '''
        # TODO(mbforbes): Implement.
        # - compute properties
        for world_object in self.world_objects:
            pass

    def _broadcast(self):
        '''
        Actually publishes the WorldObjects.
        '''
        world_object_pub.publish(self.world_objects)

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

    # Set up admin callbacks (change state of system somehow).
    admin_commands = {
        HandsFreeCommand.EXECUTE: execute_program,
        HandsFreeCommand.STOP: stop_program,
        HandsFreeCommand.CREATE_NEW_ACTION: create_new_action,
        HandsFreeCommand.SWITCH_TO: switch_to_action,
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
        self.programs = []
        self.program_idx = -1

        # Set up state broadcaster.
        self.robot_state_pub = rospy.Publisher(
            'handsfree_robotstate', RobotState)

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
        rospy.loginfo('HandsFree: Received command: ' + cmd + ' ' + str(args))

        # First, check whether executing for emergency commands.
        if self.program_idx > -1 and self.get_program().is_executing():
            if cmd not in HandsFree.emergency_commands:
                return

        # Check if admin or normal command.
        if cmd in self.admin_commands:
            # Admin: run as function.
            rospy.loginfo('HandsFree: Executing admin command: ' + cmd)
            admin_commands[cmd](self, args)
        elif cmd in HandsFree.command_map:
            # Normal: instantiate a new Command with this data.
            rospy.loginfo('HandsFree: Executing normal command: ' + cmd)
            command = HandsFree.command_map[cmd](args, phrases)

            # Execute on the robot
            code = command.execute(Mode.PROG)

            # If it worked, and we're programming, add it to the
            # program.
            if code == Code.SUCCESS and self.program_idx > -1:
                self.get_program().add_command(command)
        else:
            rospy.logwarn('HandsFree: no implementation for command: ' + cmd)

        # Always update any state changes for the parser.
        self.async_broadcast_state()

    def async_broadcast_state(self):
        '''
        Broadcasts world objects and robot state (to the parser)
        asynchronously. This is useful so that there's no pause while
        IK, etc. are computed.
        '''
        Thread(
            group=None,
            target=self.broadcast_robot_state,
            name='broadcast_robot_state_thread'
        ).start()
        Thread(
            group=None,
            target=self.maybe_broadcast_world_objects,
            name='broadcast_world_objects_thread'
        ).start()

    def maybe_broadcast_world_objects(self):
        '''
        Broadcasts world state updates to the parser IF it has
        previously been recorded.
        '''
        if self.program_idx > -1:
            self.get_program().update_world_objects()

    def broadcast_robot_state(self):
        '''
        Broadcasts robot state (to the parser).
        '''
        self.robot_state_pub.publish(self.get_robot_state())

    def get_robot_state(self):
        '''
        Gets the robot state.

        Returns:
            RobotState
        '''
        # TODO(mbforbes): Get ALL the state.
        return RobotState()

    def get_program(self):
        '''Come on, get with the program!

        Returns:
            Program|None: The current program, or None if none exists.
        '''
        return (
            self.programs[self.program_idx] if self.program_idx > -1 else None)

    # 'Admin' actions just trigger functions here.

    def execute_program(self, args):
        '''
        Executes the current program.

        Args:
            args ([str]): Command args; unused here.
        '''
        if self.program_idx > -1:
            self.get_program.execute()
        else:
            FailureFeedback.NO_PROGRAM.issue()

    def stop_program(self, args):
        '''
        Stops the current program.

        Args:
            args ([str]): Command args; unused here.
        '''
        if self.program_idx > -1 and self.get_program().is_executing():
            self.get_program.stop()
        else:
            FailureFeedback.NOT_EXECUTING.issue()

    def create_new_action(self, args):
        '''
        Creates a new program (and switches to it).

        Args:
            args ([str]): Command args; unused here.
        '''

        self.programs += [Program(GlobalOptions())]
        self.program_idx = len(self.programs) - 1

    def switch_to_action(self, args):
        '''
        Switches to the next or previous action.

        Args:
            args ([str]): Command args.
                [0] HandsFreeCommand.NEXT or HandsFreeCommand.PREVIOUS
                [1] HandsFreeCommand.ACTION
        '''
        if self.program_idx == -1:
            FailureFeedback.NO_PROGRAM_SWITCH.issue()
        elif self.program_idx == 0 and args[0] == HandsFreeCommand.PREVIOUS:
            FailureFeedback.NO_PREVIOUS.issue()
        elif (self.program_idx == len(self.programs) - 1 and
                args[0] == HandsFreeCommand.NEXT):
            FailureFeedback.NO_NEXT.issue()
        else:
            self.program_idx = (
                self.program_idx - 1 if args[0] == HandsFreeCommand.PREVIOUS
                else self.program_idx + 1)
            self.get_program().switch_to(self.program_idx + 1)  # 1-based
