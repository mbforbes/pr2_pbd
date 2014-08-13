'''Robot link for Hands-Free PbD system. Separate from PbD, living here
now for convenience.

The idea is that these classes link Hands-Free to the robot through some
backend. Currently, this is raw PR2 joint movements and IK, as well as
the pr2 object manipulation pipeline (used in PbD's world). Replacing
these with MoveIt! and other object recognition would happen in this
nmodule.
'''

__author__ = 'mbforbes'


# ######################################################################
# Imports
# ######################################################################

# Core ROS imports come first.
import roslib
roslib.load_manifest('pr2_pbd_interaction')
import rospy

# ROS Builtins
from geometry_msgs.msg import Vector3, Pose, Point

# PbD
from pr2_pbd_interaction.msg import HandsFreeCommand, Side, ArmState
from arms import Arms
from util import Numbers


# ######################################################################
# Classes
# ######################################################################

class S(object):
    '''"Singletons" or "State" or whatever you want to call it.

    This class is to hold the global vars that commands need to access.
    The fields are truely singletons, so we might as well treat them as
    so rather than passing them around everywhere.

    The name is so short because it's easier that way.
    '''
    arms = None
    world = None


class Link(object):
    '''This class provides is the interface between HandsFree and the
    rest of the PbD system. It wraps variables and converts types.

    Replacing it and S would make the system PbD-independent.
    '''
    joint_positions = {
        'side': {
            'right': [
                -0.75,  # shoulder_pan
                -0.20,  # shoulder_lift
                -2.60,  # upper_arm_roll
                -0.40,  # elbow_flex
                1.60,  # forearm_roll
                0.60,  # wrist_flex
                1.00  # wrist_roll
            ],
            'left': [
                0.75,  # shoulder_pan
                -0.20,  # shoulder_lift
                2.60,  # upper_arm_roll
                -0.40,  # elbow_flex
                -1.60,  # forearm_roll
                0.60,  # wrist_flex
                1.00  # wrist_roll
            ],
        },
    }

    # Settings.
    # TODO(mbforbes): Make launch param?
    movement_delta = 0.10  # in m, so 0.10 = 10cm (I think)

    # Constants (computed from settings).
    UP_VEC = Vector3(0.0, 0.0, movement_delta)
    DOWN_VEC = Vector3(0.0, 0.0, -movement_delta)
    LEFT_VEC = Vector3(0.0, movement_delta, 0.0)
    RIGHT_VEC = Vector3(0.0, -movement_delta, 0.0)
    FORWARD_VEC = Vector3(movement_delta, 0.0, 0.0)
    BACKWARD_VEC = Vector3(-movement_delta, 0.0, 0.0)

    abs_dir_map = {
        HandsFreeCommand.UP: UP_VEC,
        HandsFreeCommand.DOWN: DOWN_VEC,
        HandsFreeCommand.TO_LEFT: LEFT_VEC,
        HandsFreeCommand.TO_RIGHT: RIGHT_VEC,
        HandsFreeCommand.FORWARD: FORWARD_VEC,
        HandsFreeCommand.BACKWARD: BACKWARD_VEC,
    }

    @staticmethod
    def init(arms, world):
        if S.arms is None:
            S.arms = arms
        if S.world is None:
            S.world = world

    @staticmethod
    def get_gripper_joint_position(side):
        '''
        Args:
            side (int): Side.RIGHT or Side.LEFT

        Returns:
            float (0.00 <= n <= 0.08)
        '''
        return S.arms.arms[side].get_gripper_joint_position()

    @staticmethod
    def update_object_pose():
        S.world.update_object_pose()

    @staticmethod
    def refresh_objects():
        '''
        Refreshes visualization (e.g. RViz) display.
        '''
        S.world.refresh_objects()

    @staticmethod
    def get_objs():
        '''
        Returns:
            [PbdObject]
        '''
        return S.world.get_objs()

    @staticmethod
    def get_ik_for_ee(side, pose, seed):
        '''
        Args:
            side (int): Side.RIGHT or Side.LEFT
            pose (Pose): EE-position
            seed ([float]): 7-elemnt joint positions

        Returns:
            [float]|None
        '''
        return S.arms.arms[side].get_ik_for_ee(pose, seed)

    @staticmethod
    def get_gripper_state(arm_idx):
        '''
        Args:
            arm_idx (int): Side.RIGHT or Side.LEFT

        Returns:
            int: GripperState.OPEN or GripperState.CLOSED
        '''
        return S.arms.get_gripper_state(arm_idx)

    @staticmethod
    def set_gripper_state(arm_idx, gripper_state):
        '''
        Args:
            arm_idx (int): Side.RIGHT or Side.LEFT
            gripper_state (int): GripperState.OPEN or
                GripperState.CLOSED

        Returns:
            bool: Success?
        '''
        return S.arms.set_gripper_state(arm_idx, gripper_state)

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

    @staticmethod
    def get_ik_abs_dir(arm_str, abs_dir):
        '''
        Returns the joints for a a movement of arm_str in abs_dir, or
        None if it's impossible.

        Args:
            arm_str (str): HandsFreeCommand.LEFT_HAND or
                HandsFreeCommand.RIGHT_HAND
            abs_dir (str): One of:
                - HandsFreeCommand.UP
                - HandsFreeCommand.DOWN
                - HandsFreeCommand.TO_LEFT
                - HandsFreeCommand.TO_RIGHT
                - HandsFreeCommand.FORWARD
                - HandsFreeCommand.BACKWARD

        Returns:
            [float]|None: A vector of seven floats (the joint positions
                for the PR2's arm) or None if IK failed.
        '''
        arm_idx = Link.get_arm_index(arm_str)
        seed = S.arms.arms[arm_idx].get_joint_positions()
        add_vec = Link.abs_dir_map[abs_dir]
        cur_pose = S.arms.arms[arm_idx].get_ee_state()

        # We might have to fail out here if we couldn't get the EE state
        # (this happens e.g. on system startup).
        if cur_pose is None:
            return None

        # Else, compute the new position and try IK for it.
        new_pose = Pose(
            Point(
                cur_pose.position.x + add_vec.x,
                cur_pose.position.y + add_vec.y,
                cur_pose.position.z + add_vec.z
            ),
            cur_pose.orientation
        )
        rospy.loginfo('...trying to get IK for EE from arms.')
        return S.arms.arms[arm_idx].get_ik_for_ee(new_pose, seed)

    @staticmethod
    def move_to_named_position(name):
        '''
        Args:
            name (str): The name of the position to move to. See
                Link.joint_positions.

        Returns:
            bool: Whether movement was successful.

        '''
        if name not in Link.joint_positions:
            rospy.logwarn('No pre-set joint positions for ' + str(name))
            return False
        mapping = Link.joint_positions[name]
        r, l = mapping['right'], mapping['left']
        return Link.move_to_joints(r, l)

    @staticmethod
    def move_to_joints(r_joints=None, l_joints=None):
        '''
        Args:
            r_joints ([float], optional): 7-element list of joint
                positions for right arm. Defaults to None (in which case
                right arm will not move).
            l_joints ([float], optional): 7-element list of joint
                positions for left arm. Defaults to None (in which case
                left arm will not move).

        Returns:
            bool: Whether movement was successful.
        '''
        sides = [Side.RIGHT, Side.LEFT]
        targets = [r_joints, l_joints]
        arm_states = [None, None]
        for i, side in enumerate(sides):
            target = targets[i]
            if target is not None:
                cur_joints = Arms.get_joint_positions(side)
                will_move = False
                # TODO(mbforbes): Debug, this isn't actually working.
                for j in range(len(cur_joints)):
                    if not Numbers.are_floats_close(target[j], cur_joints[j]):
                        will_move = True
                        break
                if will_move:
                    arm_state = ArmState()
                    arm_state.joint_pose = target
                    arm_states[i] = arm_state

        # If we're already in the required position, don't move.
        if arm_states[0] is None and arm_states[1] is None:
            rospy.loginfo('Arms already at joints; not moving.')
            return True

        # Else, try moving.
        rospy.loginfo('Arms moving to joints...')
        return S.arms.move_to_joints(arm_states[0], arm_states[1])
