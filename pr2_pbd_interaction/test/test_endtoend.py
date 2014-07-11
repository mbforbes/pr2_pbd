#!/usr/bin/env python

'''End-to-end tests for PbD.

TODOs:
    - ensure that all Command/GuiCommands are met with the correct
    verbal response. Currently many errors could slip by. This isn't
    too hard to do.

    - check whether freezing / relaxing arms really works.
'''

# ######################################################################
# Imports
# ######################################################################

# Core ROS imports come first.
PKG = 'pr2_pbd_interaction'
import roslib
roslib.load_manifest(PKG)
import rospy

# System builtins
import sys
from collections import Counter

# ROS builtins
from sensor_msgs.msg import JointState
from actionlib import SimpleActionClient
from trajectory_msgs.msg import JointTrajectoryPoint

# ROS 3rd party
from pr2_controllers_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal
from sound_play.msg import SoundRequest

# Local
from pr2_pbd_interaction.msg import Side, GuiCommand, GripperState
from pr2_pbd_speech_recognition.msg import Command
from pr2_pbd_interaction.srv import Ping
import unittest


# ######################################################################
# Constants
# ######################################################################

# How long to wait before any tests are run to let the system start up.
# Note that we do explicitly wait until the interaction node is
# initialized, but we can't easily do this for all of the nodes that
# it depends on.
PAUSE_STARTUP = 8.0

# How long to pause when starting a new test before publishing messages.
# If we start publishing messages immediately they are dropped by ROS or
# unseen for some reason.
PAUSE_SECONDS = 2.0

# How long to allow for a command response to come in.
CMD_RESP_TIMEOUT = 2.0

# How long to wait in-between querying the recorded joint states for its
# value. Note that the joints themselves publish updates at ~90Hz.
JOINT_REFRESH_PAUSE_SECONDS = 0.1

# How long to wait in-between querying the robot speech response tracker
# for an update.
RESPONSE_REFRESH_PAUSE_SECONDS = 0.1

# Sides (right and left)
SIDES = ['r', 'l']

# Joint postfixes: gripper
GRIPPER_JOINT_POSTFIX = '_gripper_joint'

# Joint postfixes: arm
ARM_CONTROL_JOINT_POSTFIXES = [
    '_shoulder_pan_joint',
    '_shoulder_lift_joint',
    '_upper_arm_roll_joint',
    '_elbow_flex_joint',
    '_forearm_roll_joint',
    '_wrist_flex_joint',
    '_wrist_roll_joint'
]

# All postfixes
ALL_JOINT_POSTFIXES = [GRIPPER_JOINT_POSTFIX] + ARM_CONTROL_JOINT_POSTFIXES

# Combine with the side (left or right) to get full names. This is a
# single flat list (because we don't nest brackets).
RELEVANT_JOINTS = [
    side + postfix for side in SIDES for postfix in ALL_JOINT_POSTFIXES]

# Joint settings---numbers that indicate joint status. Note that there
# is +/- 0.01 error in these.
GRIPPER_OPEN_POSITION = 0.08
GRIPPER_CLOSE_POSITION = 0.00
GRIPPER_EPSILON_POSITION = 0.01
GRIPPER_TOGGLE_TIME_SECONDS = 14.0

# Most arm joints are set to this position (or its negative) when moving
# the arms around.
ARM_UP_POSITION = 0.5

# No idea what is reasonable here; PbD seems to use 0.
ARM_MOVE_VELOCITY = 0.0

# How long to pause after arm movement to let it stabilize (stop
# swinging around)
ARM_MOVE_PAUSE = 2.0

# Portions (from 0.0 to 1.0) to move the arm 'up' in a simple test
# execution.
SIMPLE_EXECUTION_PORTIONS = [0.1, 0.3, 0.6, 0.9]

# We want to mirror joint positions for the right arm.
SIDE_MULS = {'l': 1.0, "r": -1.0}

# How long to wait for each step (saved pose) in an execution, in
# seconds.
EXECUTION_STEP_TIME = 4.0

# How close arm joints have to be to match.
ARM_EPSILON_POSITION = 0.01

# ROS topics
TOPIC_INT_PING = '/interaction_ping'
TOPIC_CMD = '/recognized_command'
TOPIC_GUICMD = '/gui_command'
TOPIC_JOINTS = '/joint_states'
TOPIC_SPEECH = '/robotsound'
ARM_CONTROLLER_POSTFIX = '_arm_controller/joint_trajectory_action'


# ######################################################################
# Classes
# ######################################################################

class TestEndToEnd(unittest.TestCase):
    '''End-to-end tests for PbD.'''

    # ##################################################################
    # Core test infrastructure
    # ##################################################################

    def setUp(self):
        '''Ensures there is a valid interaction instance and wait until
        it's ready to rumble.'''
        # Ensure the interaction node is ready.
        rospy.wait_for_service('/interaction_ping')

        # Initialize some state.
        self.joint_positions = {}
        for joint in RELEVANT_JOINTS:
            self.joint_positions[joint] = None

        # Set up map of arm control joint names.
        self.arm_control_joints = {}
        for side in SIDES:
            self.arm_control_joints[side] = [
                side + postfix for postfix in ARM_CONTROL_JOINT_POSTFIXES]

        # Set up Counter to track robot speech.
        self.speech_tracker = Counter()

        # Create our ROS message machinery.
        # Keep alive ping.
        self.ping_srv = rospy.ServiceProxy(TOPIC_INT_PING, Ping)
        # For publishing speech/GUI commands.
        self.command_pub = rospy.Publisher(TOPIC_CMD, Command)
        # For publishing GUI-only commands.
        self.gui_command_pub = rospy.Publisher(TOPIC_GUICMD, GuiCommand)
        # For tracking gripper/arm states.
        rospy.Subscriber(TOPIC_JOINTS, JointState, self.joint_states_cb)
        # For tracking robot speech (its responses).
        rospy.Subscriber(TOPIC_SPEECH, SoundRequest, self.speech_heard_cb)
        # Set up controllers to move arms.
        self.arm_controllers = {}
        for side in SIDES:
            controller_name = '/' + side + ARM_CONTROLLER_POSTFIX
            self.arm_controllers[side] = SimpleActionClient(
                controller_name, JointTrajectoryAction)
            rospy.loginfo('Waiting for ' + side + ' arm server.')
            self.arm_controllers[side].wait_for_server()
            rospy.loginfo('Got response from ' + side + ' arm server.')

        # Apparently need to wait a bit even here, or messages get
        # dropped.
        rospy.sleep(PAUSE_SECONDS)

    # ##################################################################
    # Tests
    # ##################################################################

    def test_a_noaction_branches(self):
        '''This ideally exercises the "sorry, no action created yet"
        code that prevents requests from going through.

        The name "_a_" is in this test so that it runs first. This is
        because in PbD, once you create actions, you can never delete
        them, and it's not worth tearing down / bringing up PbD for
        each test case (it's also difficult to launch ROS nodes from
        within testing code).

        The one solace for this "test should run first" test, which is
        typically bad practice (and using "_a_" to make it run first,
        which is test runner-dependent), is that this test will still
        pass if other tests run first; it merely won't exercise the
        code that it's aiming to.
        '''
        # Ensure things are ready to go.
        self.check_alive()

        # Switch to nonexistant action
        self.gui_command_pub.publish(
            GuiCommand(GuiCommand.SWITCH_TO_ACTION, 50))

        # Switch to nonexistant step
        self.gui_command_pub.publish(
            GuiCommand(GuiCommand.SELECT_ACTION_STEP, 50))

        # Switch around actions
        self.command_pub.publish(Command(Command.NEXT_ACTION))
        self.command_pub.publish(Command(Command.PREV_ACTION))

        # Do naughty things within nonexistant action.
        self.command_pub.publish(Command(Command.DELETE_LAST_STEP))
        self.command_pub.publish(Command(Command.DELETE_ALL_STEPS))
        self.command_pub.publish(Command(Command.START_RECORDING_MOTION))
        self.command_pub.publish(Command(Command.SAVE_POSE))
        # No explicit check for record object pose, but it doesn't hurt
        self.command_pub.publish(Command(Command.RECORD_OBJECT_POSE))
        self.command_pub.publish(Command(Command.EXECUTE_ACTION))

        # Now make a single action and try bad switches.
        self.command_pub.publish(Command(Command.CREATE_NEW_ACTION))
        self.command_pub.publish(Command(Command.NEXT_ACTION))
        self.command_pub.publish(Command(Command.PREV_ACTION))

        # Make sure nothing's crashed.
        self.check_alive()

    def test_stop_execution(self):
        '''Test name says it all. Extremely simple.'''
        # Ensure things are ready to go.
        self.check_alive()

        # Stop execution while not executing, no steps.
        self.command_pub.publish(Command(Command.CREATE_NEW_ACTION))
        self.command_pub.publish(Command(Command.STOP_EXECUTION))

        # Try starting and stopping (shouldn't run as no steps).
        self.command_pub.publish(Command(Command.EXECUTE_ACTION))
        self.command_pub.publish(Command(Command.STOP_EXECUTION))

        # Make some steps, "stop.""
        for i in range(4):
            self.command_pub.publish(Command(Command.SAVE_POSE))
        self.command_pub.publish(Command(Command.STOP_EXECUTION))

        # Now actually start executing and stop.
        self.command_pub.publish(Command(Command.EXECUTE_ACTION))
        self.command_pub.publish(Command(Command.STOP_EXECUTION))

        # Make sure nothing's crashed.
        self.check_alive()

    def test_freeze_relax_arm(self):
        '''Tests the freeze and relax arm functionality.'''
        # Ensure things are ready to go.
        self.check_alive()

        # Relax both
        self.command_pub.publish(Command(Command.RELAX_RIGHT_ARM))
        self.command_pub.publish(Command(Command.RELAX_LEFT_ARM))

        # Relax *again*
        self.command_pub.publish(Command(Command.RELAX_RIGHT_ARM))
        self.command_pub.publish(Command(Command.RELAX_LEFT_ARM))

        # Freeze both
        self.command_pub.publish(Command(Command.FREEZE_RIGHT_ARM))
        self.command_pub.publish(Command(Command.FREEZE_LEFT_ARM))

        # Freeze *again*
        self.command_pub.publish(Command(Command.FREEZE_RIGHT_ARM))
        self.command_pub.publish(Command(Command.FREEZE_LEFT_ARM))

        # Make sure nothing's crashed.
        self.check_alive()

    def test_gripper_open_close(self):
        '''Tests that issuing 'speech' commands to open and close the
        gripper puts them in the desired state.'''
        # Ensure things are ready to go.
        self.check_alive()

        # Check opening/closing hands works
        self.command_pub.publish(Command(Command.OPEN_RIGHT_HAND))
        self.command_pub.publish(Command(Command.OPEN_LEFT_HAND))
        for joint in [side + GRIPPER_JOINT_POSTFIX for side in SIDES]:
            self.assertJointCloseWithinTimeout(
                joint,
                GRIPPER_OPEN_POSITION,
                GRIPPER_EPSILON_POSITION,
                GRIPPER_TOGGLE_TIME_SECONDS
            )
        self.command_pub.publish(Command(Command.CLOSE_RIGHT_HAND))
        self.command_pub.publish(Command(Command.CLOSE_LEFT_HAND))
        for joint in [side + GRIPPER_JOINT_POSTFIX for side in SIDES]:
            self.assertJointCloseWithinTimeout(
                joint,
                GRIPPER_CLOSE_POSITION,
                GRIPPER_EPSILON_POSITION,
                GRIPPER_TOGGLE_TIME_SECONDS
            )

        # Make sure nothing's crashed.
        self.check_alive()

    def test_double_open_close(self):
        '''Tests that issuing a gripper open / close command twice
        doesn't break the robot and it stays in the desired state.
        '''
        # Ensure things are ready to go.
        self.check_alive()

        # Test settings (to avoid code duplication)
        positions = [GRIPPER_OPEN_POSITION, GRIPPER_CLOSE_POSITION]
        right_commands = [Command.OPEN_RIGHT_HAND, Command.CLOSE_RIGHT_HAND]
        left_commands = [Command.OPEN_LEFT_HAND, Command.CLOSE_LEFT_HAND]

        # Check each opening & closing, and do each twice.
        for state_idx in range(len(positions)):
            # First open/close once.
            self.command_pub.publish(Command(right_commands[state_idx]))
            self.command_pub.publish(Command(left_commands[state_idx]))
            for joint in [side + GRIPPER_JOINT_POSTFIX for side in SIDES]:
                self.assertJointCloseWithinTimeout(
                    joint,
                    positions[state_idx],
                    GRIPPER_EPSILON_POSITION,
                    GRIPPER_TOGGLE_TIME_SECONDS
                )
            # Then, do the same thing again.
            self.command_pub.publish(Command(right_commands[state_idx]))
            self.command_pub.publish(Command(left_commands[state_idx]))
            for joint in [side + GRIPPER_JOINT_POSTFIX for side in SIDES]:
                # Note that here we do "After" to ensure it doesn't just
                # pass immediately. But we don't wait the full time
                # (half is plenty to detect any change).
                self.assertJointCloseAfter(
                    joint,
                    positions[state_idx],
                    GRIPPER_EPSILON_POSITION,
                    GRIPPER_TOGGLE_TIME_SECONDS / 2.0
                )

        # Make sure nothing's crashed.
        self.check_alive()

    def test_action_and_step_navigation(self):
        '''Tests creating / switching between actions, and creating /
        switching between steps.

        We could expose APIs within the system to query its state after
        issuing these commands, but at this point we're just going to
        exercise them and make sure things haven't crashed.
        '''
        # Ensure things are ready to go.
        self.check_alive()

        # Create / switch between actions.
        self.command_pub.publish(Command(Command.CREATE_NEW_ACTION))
        self.command_pub.publish(Command(Command.CREATE_NEW_ACTION))
        self.command_pub.publish(Command(Command.PREV_ACTION))
        self.command_pub.publish(Command(Command.NEXT_ACTION))

        # Try deleting last / all poses when there are none.
        self.command_pub.publish(Command(Command.DELETE_LAST_STEP))
        self.command_pub.publish(Command(Command.DELETE_ALL_STEPS))

        # Make some steps.
        self.command_pub.publish(Command(Command.SAVE_POSE))
        self.command_pub.publish(Command(Command.SAVE_POSE))
        self.command_pub.publish(Command(Command.SAVE_POSE))
        self.command_pub.publish(Command(Command.SAVE_POSE))

        # Switch between the steps.
        self.gui_command_pub.publish(
            GuiCommand(GuiCommand.SELECT_ACTION_STEP, 3))
        self.gui_command_pub.publish(
            GuiCommand(GuiCommand.SELECT_ACTION_STEP, 2))
        self.gui_command_pub.publish(
            GuiCommand(GuiCommand.SELECT_ACTION_STEP, 4))
        self.gui_command_pub.publish(
            GuiCommand(GuiCommand.SELECT_ACTION_STEP, 1))

        # Navigate away/to the action, switch to a step
        self.command_pub.publish(Command(Command.PREV_ACTION))
        self.command_pub.publish(Command(Command.NEXT_ACTION))
        self.gui_command_pub.publish(
            GuiCommand(GuiCommand.SELECT_ACTION_STEP, 2))

        # Delete a step and try to switch to it
        self.command_pub.publish(Command(Command.DELETE_LAST_STEP))
        self.gui_command_pub.publish(
            GuiCommand(GuiCommand.SELECT_ACTION_STEP, 4))

        # Delete all steps and try to switch to one
        self.command_pub.publish(Command(Command.DELETE_ALL_STEPS))
        self.gui_command_pub.publish(
            GuiCommand(GuiCommand.SELECT_ACTION_STEP, 2))

        # Final switching (depending on test run order, may switch to
        # different actions than we created here; hence, this comes last
        # in the test so we don't much with previously created steps.
        # Not that it matters... but nothing here should depend on the
        # existence or non-existence of steps).
        self.gui_command_pub.publish(
            GuiCommand(GuiCommand.SWITCH_TO_ACTION, 1))
        self.gui_command_pub.publish(
            GuiCommand(GuiCommand.SELECT_ACTION_STEP, 2))
        self.gui_command_pub.publish(
            GuiCommand(GuiCommand.SWITCH_TO_ACTION, 1))

        # Make sure nothing's crashed.
        self.check_alive()

    def test_simple_execution(self):
        '''Extremely simple execution test: just save poses in place and
        execute. Merely testing lack of system crash.'''
        # Ensure things are ready to go.
        self.check_alive()

        # Make an action and one pose.
        self.command_pub.publish(Command(Command.CREATE_NEW_ACTION))
        self.command_pub.publish(Command(Command.SAVE_POSE))

        # Executing here should say "no" because there aren't enough
        # poses.
        self.command_pub.publish(Command(Command.EXECUTE_ACTION))

        # Make an action and execute. It should work now.
        self.command_pub.publish(Command(Command.SAVE_POSE))
        self.command_pub.publish(Command(Command.EXECUTE_ACTION))

        # Make sure nothing's crashed.
        self.check_alive()

    def test_moving_execution(self):
        '''Test moving the arms a few times and executing.'''
        # Ensure things are ready to go.
        self.check_alive()

        self.command_pub.publish(Command(Command.CREATE_NEW_ACTION))

        # Move arms to several different increments, saving each time.
        for portion in SIMPLE_EXECUTION_PORTIONS:
            self.move_arms_up(portion)
            self.command_pub.publish(Command(Command.SAVE_POSE))

        # Move arms to bottom position.
        self.move_arms_up(0.0)

        # Execute!
        self.command_pub.publish(Command(Command.EXECUTE_ACTION))

        # Make sure the arms get there by checking one of the joints.
        side = SIDES[0]
        joint_name = self.arm_control_joints[side][0]
        expected_position = (
            SIDE_MULS[side] * ARM_UP_POSITION * SIMPLE_EXECUTION_PORTIONS[-1])
        wait_time = EXECUTION_STEP_TIME * len(SIMPLE_EXECUTION_PORTIONS)
        self.assertJointCloseWithinTimeout(
            joint_name, expected_position, ARM_EPSILON_POSITION, wait_time)

        # Make sure nothing's crashed.
        self.check_alive()

    def test_open_close_execution(self):
        '''Test moving the arms a few times and saving poses with open/close
        hand, then executing.'''
        # Ensure things are ready to go.
        self.check_alive()

        # Make sure grippers are open, THEN create new action.
        self.command_pub.publish(Command(Command.OPEN_RIGHT_HAND))
        self.command_pub.publish(Command(Command.OPEN_LEFT_HAND))
        self.command_pub.publish(Command(Command.CREATE_NEW_ACTION))

        # Move arms to several different increments, opening / closing
        # each time to save two poses.
        last_toggle = 'open'
        gripper_cmds = {
            'open': [
                Command(Command.OPEN_RIGHT_HAND),
                Command(Command.OPEN_LEFT_HAND)
            ], 'close': [
                Command(Command.CLOSE_RIGHT_HAND),
                Command(Command.CLOSE_LEFT_HAND)
            ]
        }
        for portion in SIMPLE_EXECUTION_PORTIONS:
            # Move the arms
            self.move_arms_up(portion)

            # Save two poses by opening/closing each hand
            last_toggle = 'close' if last_toggle == 'open' else 'open'
            cmds = gripper_cmds[last_toggle]
            for cmd in cmds:
                self.command_pub.publish(cmd)

        # Move arms to bottom position.
        self.move_arms_up(0.0)

        # Execute!
        self.command_pub.publish(Command(Command.EXECUTE_ACTION))

        # Make sure the arms get there by checking one of the joints.
        side = SIDES[0]
        joint_name = self.arm_control_joints[side][0]
        expected_position = (
            SIDE_MULS[side] * ARM_UP_POSITION * SIMPLE_EXECUTION_PORTIONS[-1])
        # Multiply by two as we saved two poses per each move
        wait_time = EXECUTION_STEP_TIME * len(SIMPLE_EXECUTION_PORTIONS) * 2
        self.assertJointCloseWithinTimeout(
            joint_name, expected_position, ARM_EPSILON_POSITION, wait_time)

        # Also check grippers in desired state. Because the execution
        # involves opening and closing hands each time, they might still
        # need the full time to open/close here (as moving arms is much
        # faster).
        expected_gripper_position = (
            GRIPPER_OPEN_POSITION if last_toggle == 'open'
            else GRIPPER_CLOSE_POSITION)
        for joint in [side + GRIPPER_JOINT_POSTFIX for side in SIDES]:
            self.assertJointCloseWithinTimeout(
                joint,
                expected_gripper_position,
                GRIPPER_EPSILON_POSITION,
                GRIPPER_TOGGLE_TIME_SECONDS
            )

        # Make sure nothing's crashed.
        self.check_alive()

    # ##################################################################
    # Helper methods
    # ##################################################################

    def check_alive(self):
        '''Ensures the interaction node is alive.'''
        self.ping_srv()
        self.assertTrue(True, "Interaction node should be alive.")

    def speech_heard_cb(self, sound_request):
        '''Called when the robot requests speech; tracks responses.

        Args:
            sound_request (SoundRequest): Request robot sent for speech
                to be played.
        '''
        # We only track things the robot is going to say (text).
        if sound_request.sound == SoundRequest.SAY:
            # Text is passed in arg.
            text = sound_request.arg
            self.speech_tracker[text] += 1

    def joint_states_cb(self, joint_state):
        '''Tracks relevant joint states.

        Args:
            joint_state (JointState): Sensor messages published by the
                PR2, reporting the state of its joints.
        '''
        names = joint_state.name
        positions = joint_state.position
        for joint in RELEVANT_JOINTS:
            if joint in names:
                self.joint_positions[joint] = positions[names.index(joint)]

    def cmd_assert_response(self, command, responses, timeout):
        '''Issues a command and asserts that one of the valid responses
        is heard within timout seconds.

        Args:
            command (str): One of Command.*
            responses ([str]): Each element is one of RobotSpeech.*
            timeout (float): How many seconds to wait before failing.
        '''
        # Get number of times response was heard previously.
        prev_resp_map = {}
        for response in responses:
            prev_resp_map[response] = self.speech_tracker[response]

        # Issue the command
        self.command_pub.publish(Command(command))

        # Check if response count increased once before waiting for
        # timeout.
        if self.any_resp_inc(prev_resp_map, responses):
            return

        # Wait for timeout, checking if response heard.
        timeout_dur = rospy.Duration(timeout)
        start = rospy.Time.now()
        while rospy.Time.now() - start < timeout_dur:
            if self.any_resp_inc(prev_resp_map, responses):
                return
            rospy.sleep(RESPONSE_REFRESH_PAUSE_SECONDS)

        # Check one last time before failing.
        if self.any_resp_inc(prev_resp_map, responses):
            return

        # Not heard! Fail.
        self.assertFalse(
            True,
            "Never heard expected response: %s" %
            (response)
        )

    def any_resp_inc(self, prev_resp_map, responses):
        '''Returns whether any response in responses was seen exaclty
        once more than is recorded in prev_resp_map.

        Args:
            prev_resp_map ({str: int}): Map of previous responses to
                their counts.
            responses ([str]): Any of the possible responses that could
                have been heard.
        '''
        for response in responses:
            # Must match exactly one greater than recorded previously.
            if self.speech_tracker[response] == prev_resp_map[response] + 1:
                return True
        # None match.
        return False

    def are_floats_close(self, a, b, epsilon):
        '''Checks whether two floats are within epsilon of each
        other.

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

    def move_arms_up(self, portion=1.0):
        '''This method moves the arms 'up', meaning most seven arm
        joints get twisted by some amount until they're moderately 'up'.
        The portion is how much towards this 'up' position to go to,
        where 0.0 is arms in front, and 1.0 is arms fully up.

        Args:
            portion (float): 0.0 <= portion <= 1.0. How much of the way
                up to move the arms.
        '''
        # Safety check.
        portion = 0.0 if portion <= 0.0 else portion
        portion = 1.0 if portion >= 1.0 else portion

        # We want to mirror some of the numbers across 0 for each arm.

        for side in SIDES:
            # For convenience.
            joints = self.arm_control_joints[side]
            # We have some specific configurations for the joints:
            # - The the main shoulder pan joint (first) should be
            #       mirrored (so that arms swing out)
            # - The shoulder lift joint (second) should always be
            #       reversed (so arms both go up)
            # - The upper arm roll joint (third) should be mirrored, as
            #       normal.
            # - The elbow flex joint should be unmirrored and always set
            #       to the fully up position. This helps the poses that
            #       are generated by 'reachable' by bringing them closer
            #       to the robot's body.
            # - The remaining joints are mirrored, as normal
            pan_el = [ARM_UP_POSITION * SIDE_MULS[side] * portion]
            lift_el = [ARM_UP_POSITION * -1.0 * portion]
            uproll_el = [ARM_UP_POSITION * portion]
            elflex_el = [ARM_UP_POSITION]
            other_els = (
                [ARM_UP_POSITION * SIDE_MULS[side] * portion] *
                (len(joints) - 4))
            positions = pan_el + lift_el + uproll_el + elflex_el + other_els
            velocities = [ARM_MOVE_VELOCITY] * len(joints)

            goal = JointTrajectoryGoal()
            goal.trajectory.joint_names = joints
            goal.trajectory.points.append(JointTrajectoryPoint(
                positions=positions, velocities=velocities))
            self.arm_controllers[side].send_goal(goal)
            self.arm_controllers[side].wait_for_result()

        # Let arms stop shaking around.
        rospy.sleep(ARM_MOVE_PAUSE)

    def assertJointCloseAfter(
            self, joint_name, expected_val, epsilon, time):
        '''Asserts that the position of the joint given by joint_name
        is "close to" expected_val after time seconds.

        Note that this is different than "assertJointCloseWithinTimeout"
        as this method waits to assert until after time. This is useful
        if, for instance, you want to check something remains close to
        a value after a period of time.

        (Note that this method is named in camelCase to match the other
        assert methods in the unittest library).

        Args:
            joint_name (str): Name of the joint to query.
            expected_val (float): Position joint must reach "close to".
            epsilon (float): Wiggle room (+/-) of joint reaching
                expected_val.
            time (float): How many seconds to wait before asserting the
                joint is close to the expected_val position.
        '''
        rospy.sleep(time)
        self.assertTrue(
            self.are_floats_close(
                self.joint_positions[joint_name], expected_val, epsilon),
            "Joint %s isn't at its expected value %f" %
            (joint_name, expected_val)
        )

    def assertJointCloseWithinTimeout(
            self, joint_name, expected_val, epsilon, timeout):
        '''Asserts that the position of the joint given by joint_name
        reaches "close to" expected_val within timeout seconds.

        (Note that this method is named in camelCase to match the other
        assert methods in the unittest library).

        Args:
            joint_name (str): Name of the joint to query.
            expected_val (float): Position joint must reach "close to".
            epsilon (float): Wiggle room (+/-) of joint reaching
                expected_val.
            timeout (float): How many seconds the joint has to reach
                close to the expected_val position.
        '''
        # We check this generously by checking once before and after the
        # timeout.
        if self.are_floats_close(
                self.joint_positions[joint_name], expected_val, epsilon):
            return

        # Check / sleep through timeout
        timeout_dur = rospy.Duration(timeout)
        start = rospy.Time.now()
        while rospy.Time.now() - start < timeout_dur:
            if self.are_floats_close(
                    self.joint_positions[joint_name], expected_val, epsilon):
                return
            rospy.sleep(JOINT_REFRESH_PAUSE_SECONDS)

        # Check once after timeout.
        if self.are_floats_close(
                self.joint_positions[joint_name], expected_val, epsilon):
            return

        # Didn't make it; fail the test!
        self.assertFalse(
            True,
            "Joint %s never reached its expected value %f" %
            (joint_name, expected_val)
        )

# ######################################################################
# Program execution begins here
# ######################################################################

if __name__ == '__main__':
    rospy.init_node('test_endtoend')
    import rostest
    rospy.loginfo("Running tests in simulation.")
    rospy.loginfo("Waiting for system to start up.")
    rospy.sleep(PAUSE_STARTUP)
    rospy.loginfo("Done waiting for system to start up.")
    rostest.rosrun(
        PKG,  # package_name
        'test_end_to_end',  # test_name
        TestEndToEnd,  # test_case_class
    )
