'''Robot state manager for Hands-Free PbD system. Separate from PbD,
living here now for convenience.'''

__author__ = 'mbforbes'


# ######################################################################
# Imports
# ######################################################################

# Core ROS imports come first.
import roslib
roslib.load_manifest('pr2_pbd_interaction')
import rospy

# System builtins
from threading import Thread

# Pbd (3rd party / local)
from pr2_pbd_interaction.msg import HandsFreeCommand, RobotState, Side

# Local
from robotlink import Link


# ######################################################################
# Classes
# ######################################################################

class RobotHandler(object):
    '''Manages robot state.'''

    # Set up robot state broadcaster.
    robot_state_pub = rospy.Publisher('handsfree_robotstate', RobotState)

    # Where we store the state.
    robot_state = RobotState()

    # Where we store the last-commanded side. Commands sets this.
    last_commanded = RobotState.NEITHER

    # Where we store whether we are executing. Program sets this.
    is_executing = False

    @staticmethod
    def async_broadcast():
        '''
        Updates robot state and broadcasts it (e.g. to the parser).

        Non-blocking.
        '''
        Thread(
            group=None,
            target=RobotHandler.broadcast,
            name='broadcast_robot_state_thread'
        ).start()

    @staticmethod
    def broadcast():
        '''
        Updates robot state and broadcasts it (e.g. to the parser).

        Blocking.
        '''
        RobotHandler._update()
        RobotHandler.robot_state_pub.publish(RobotHandler.robot_state)

    @staticmethod
    def _update():
        '''
        Computes and sets the robot state.
        '''
        # Set vars
        side_names = [HandsFreeCommand.RIGHT_HAND, HandsFreeCommand.LEFT_HAND]

        # Create empty
        rs = RobotState()

        # Fill in basic vals.
        rs.last_cmd_side = RobotHandler.last_commanded
        rs.is_executing = RobotHandler.is_executing

        # Fill in gripper states.
        for side in [Side.RIGHT, Side.LEFT]:
            val = Link.get_gripper_joint_position(side)
            # Heuristic: Judege based on how open/close the gripper is.
            # This will fail, for example, when picking up a thin
            # object.
            # TODO(mbforbes): Refactor into constants.
            if val <= 0.02:
                state = RobotState.CLOSED_EMPTY
            elif val >= 0.078:
                state = RobotState.OPEN
            else:
                state = RobotState.HAS_OBJ
            rs.gripper_states += [state]

        # Fill in via IK.
        rs.can_move_up = [
            Link.get_ik_abs_dir(s, HandsFreeCommand.UP) is not None
            for s in side_names]
        rs.can_move_down = [
            Link.get_ik_abs_dir(s, HandsFreeCommand.DOWN) is not None
            for s in side_names]
        rs.can_move_toleft = [
            Link.get_ik_abs_dir(s, HandsFreeCommand.TO_LEFT) is not None
            for s in side_names]
        rs.can_move_toright = [
            Link.get_ik_abs_dir(s, HandsFreeCommand.TO_RIGHT) is not None
            for s in side_names]
        rs.can_move_forward = [
            Link.get_ik_abs_dir(s, HandsFreeCommand.FORWARD) is not None
            for s in side_names]
        rs.can_move_backward = [
            Link.get_ik_abs_dir(s, HandsFreeCommand.BACKWARD) is not None
            for s in side_names]

        # Set.
        RobotHandler.robot_state = rs
