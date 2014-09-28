'''Hands-Free PbD system. Separate from PbD, living here now for
convenience.'''

__author__ = 'mbforbes'


# ######################################################################
# Imports
# ######################################################################

# Core ROS imports come first.
import roslib
roslib.load_manifest('pr2_pbd_interaction')
import rospy

# PbD (3rd party / local)
from pr2_pbd_interaction.msg import HandsFreeCommand, Description

# True local
from feedback import Feedback, FailureFeedback
from robot import RobotHandler
from robotlink import Link
from program import Program
from commands import CommandRouter, Mode, Code
from objects import ObjectsHandler
from util import GlobalOptions, Logger


# ######################################################################
# Classes
# ######################################################################

class HandsFree(object):
    '''Sets up the hands-free system.'''

    FF_NO_PROGRAM = FailureFeedback('No actions to execute.')
    FF_NO_PROGRAM_SWITCH = FailureFeedback('No actions created to switch to.')
    FF_NOT_EXECUTING = FailureFeedback('Not executing action; cannot stop.')
    FF_NO_PREVIOUS = FailureFeedback('No previous action.')
    FF_NO_NEXT = FailureFeedback('No next action.')

    def __init__(self, arms, world):
        # Setup robot link (middleware).
        Link.init(arms, world)

        # Set up state.
        self.programs = []
        self.program_idx = -1

        # Set up the command and description dispatch.
        rospy.Subscriber(
            'handsfree_command', HandsFreeCommand, self.command_cb)
        rospy.Subscriber(
            'handsfree_description', Description, self.description_cb)

        # Send off one robot state to get system started.
        # NOTE(mbforbes): Once the system is working, it might be best
        # to just create a new action (and look for objects) right away
        # so people can just start programming right off the bat.
        RobotHandler.async_broadcast()

    def cleanup(self):
        '''
        Called as the system is exiting.
        '''
        Logger.L.cleanup()

    def description_cb(self, desc):
        '''
        Callback for when praser sends us a description of the world's
        objects.

        Args:
            desc (Description)
        '''
        # Record
        Logger.L.save_desc(desc)
        # Send to objects handler for describing.
        names, descs = desc.object_names, desc.descriptions
        ObjectsHandler.save_descriptions(names, descs)

    def command_cb(self, hf_cmd):
        '''
        Callback for when praser sends us a command.

        Args:
            hf_cmd (HandsFreeCommand)
        '''
        # Log
        Logger.L.save_cmd(hf_cmd)

        # Extract vars for ease of use
        cmd, args, phrases = hf_cmd.cmd, hf_cmd.args, hf_cmd.phrases
        rospy.loginfo('HandsFree: Received command: ' + cmd + ' ' + str(args))

        # First, check whether executing for emergency commands.
        if Program.is_executing():
            if cmd not in CommandRouter.emergency_commands:
                return

        # Check if admin or normal command.
        if cmd in CommandRouter.admin_commands:
            # Admin: run as function.
            rospy.loginfo('HandsFree: Executing admin command: ' + cmd)
            getattr(self, CommandRouter.admin_commands[cmd])(args, phrases)
        elif cmd in CommandRouter.command_map:
            # Normal: instantiate a new Command with this data.
            rospy.loginfo('HandsFree: Executing normal command: ' + cmd)
            command = CommandRouter.command_map[cmd](args, phrases)

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
        RobotHandler.async_broadcast()
        # World objects don't change, so broadcasting is fast.
        ObjectsHandler.broadcast()

    def get_program(self):
        '''Come on, get with the program!

        Returns:
            Program|None: The current program, or None if none exists.
        '''
        return (
            self.programs[self.program_idx] if self.program_idx > -1 else None)

    # 'Admin' actions just trigger functions here.

    def clarify(self, args, phrases):
        '''
        Ask user to clarify intent.

        Args:
            args ([str]): Contains arguments that need to be clarified,
                if the top-scoring commands are all of the same
                template. Otherwise, blank (is this best?).
            phrases ([str]): Command phrases; unused here.
        '''
        # TODO(mbforbes): Be helpful.
        if len(args) > 0:
            arg_clar = ' and'.join(args)
            fb = Feedback('Please clarify ' + arg_clar)
        else:
            fb = Feedback('Please rephrase command.')
        fb.issue()

    def execute_program(self, args, phrases):
        '''
        Executes the current program.

        Args:
            args ([str]): Command args; unused here.
            phrases ([str]): Command phrases; unused here.
        '''
        if self.program_idx > -1:
            self.get_program().execute()
        else:
            HandsFree.FF_NO_PROGRAM.issue()

    def stop_program(self, args, phrases):
        '''
        Stops the current program.

        Args:
            args ([str]): Command args; unused here.
            phrases ([str]): Command phrases; unused here.
        '''
        if Program.is_executing():
            self.get_program().stop()
        else:
            HandsFree.FF_NOT_EXECUTING.issue()

    def create_new_action(self, args, phrases):
        '''
        Creates a new program (and switches to it).

        Args:
            args ([str]): Command args; unused here.
            phrases ([str]): Command phrases; unused here.
        '''
        next_idx = len(self.programs)
        self.programs += [
            Program(
                GlobalOptions(),
                next_idx + 1  # 1-based index
            )
        ]
        self.program_idx = next_idx

    def switch_to_action(self, args, phrases):
        '''
        Switches to the next or previous action.

        Args:
            args ([str]): Command args.
                [0] HandsFreeCommand.NEXT or HandsFreeCommand.PREVIOUS
                [1] HandsFreeCommand.ACTION
            phrases ([str]): Command phrases; unused here.
        '''
        if self.program_idx == -1:
            HandsFree.FF_NO_PROGRAM_SWITCH.issue()
        elif self.program_idx == 0 and args[0] == HandsFreeCommand.PREVIOUS:
            HandsFree.FF_NO_PREVIOUS.issue()
        elif (self.program_idx == len(self.programs) - 1 and
                args[0] == HandsFreeCommand.NEXT):
            HandsFree.FF_NO_NEXT.issue()
        else:
            self.program_idx = (
                self.program_idx - 1 if args[0] == HandsFreeCommand.PREVIOUS
                else self.program_idx + 1)
            self.get_program().switch_to(self.program_idx + 1)  # 1-based
