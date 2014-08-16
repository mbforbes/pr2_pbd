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
from actionlib import SimpleActionClient
from geometry_msgs.msg import Vector3, Pose, Point
import tf

# ROS 3rd party
from pr2_object_manipulation_msgs.msg import (
    IMGUIAction, IMGUIActionGoal, IMGUIActionResult, IMGUIGoal, IMGUIOptions,
    IMGUICommand, IMGUIAdvancedOptions)
from object_manipulation_msgs.msg import ManipulationResult

# PbD
from pr2_pbd_interaction.msg import HandsFreeCommand, Side, ArmState
from arms import Arms
from world import World
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
    imgui_action_client = None


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

    # ##################################################################
    # Public static methods (should hide IK vs MoveIt! implementation)
    # ##################################################################

    @staticmethod
    def init(arms, world):
        if S.arms is None:
            S.arms = arms
        if S.world is None:
            S.world = world
        if S.imgui_action_client is None:
            S.imgui_action_client = SimpleActionClient(
                'imgui_action',  # namespace
                IMGUIAction  # action message type
            )


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
    def get_computed_pose_possible(side, pose):
        '''
        Returns whether a pose that was determined programmatically
        (i.e. NOT relative to known robot EE-pose) is reachable.

        Args:
            side (int): Side.RIGHT or Side.LEFT
            pose (Pose)

        Returns:
            bool
        '''
        return Link._get_ik_for_ee_computed(side, pose) is not None

    @staticmethod
    def pick_up(obj, side):
        '''
        Args:
            obj (PbdObject)
            side (int): Side.RIGHT or Side.LEFT
        '''
        # See the following for documentation of these
        # objects (and links to their spec).
        # https://docs.google.com/document/d/
        #     10Pqi5M2aKWgubR7UMSLoYJhApbD3fhWq-9C7yrxvdpA/
        #     edit#
        adv_opt = IMGUIAdvancedOptions(
            False,  # reactive_grasping
            False,  # reactive_force
            False,  # reactive_place
            10,  # lift_steps
            10,  # retreat_steps
            0,  # lift_direction_choice
            0,  # desired_approach
            0,  # min_approach
            0.0,  # max_contact_force # TODO(max): 30?
            False,  # find_alternatives
            False,  # always_plan_grasps
            False  # cycle_gripper_opening
        )
        opts = IMGUIOptions(
            True,  # collision_checked
            1,  # grasp_selection (0 grip click, 1 provided)
            0,  # arm_selection (0 right, 1 left)
            0,  # reset_choice (reset: 0 collision, 1 attached)
            0,  # arm_action_choice (0 side, 1 front, 2 side handoff)
            0,  # arm_planner_choice (0 open-loop, 1 w/ planner)
            0,  # gripper_slider_position (0=closed...100=open)
            obj.gobj,  # selected_object (GraspableObject)
            [],  # moveable_obstacles
            adv_opt  # adv_options
        )
        cmd = IMGUICommand(
            0,  # 0 pickup, 1 place, 2 planned move, etc.
            '',  # script_name
            ''  # script_group_name
        )
        goal = IMGUIGoal(opts, cmd)
        rospy.loginfo('Sending pickup goal')
        S.imgui_action_client.send_goal(goal)
        S.imgui_action_client.wait_for_result()
        return (
            S.imgui_action_client.get_result().result.value ==
            ManipulationResult.SUCCESS)

    @staticmethod
    def move_to_computed_pose(side, pose):
        '''
        Args:
            side (int): Side.RIGHT or Side.LEFT
            pose (Pose)

        Returns:
            bool: Whether movement was successful.
        '''
        side_joints = Link._get_ik_for_ee_computed(side, pose)

        # Check whether possible
        if side_joints is None:
            return False

        # Possible; return movement success.
        joints = [None, None]
        joints[side] = side_joints
        return Link._move_to_joints(joints[0], joints[1])

    @staticmethod
    def get_abs_dir_possible(arm_str, abs_dir):
        '''
        Returns whether a movement of arm_str in abs_dir is possible.

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
            bool
        '''
        new_pose = Link._get_pose_abs_dir(arm_str, abs_dir)

        # If we're not initialized yet (early in startup of system),
        # then we must just return false.
        if new_pose is None:
            return False

        # Else, give it a shot.
        arm_idx = Link.get_arm_index(arm_str)
        seed = S.arms.arms[arm_idx].get_joint_positions()
        return Link._get_ik_for_ee_raw(arm_idx, new_pose, seed) is not None

    @staticmethod
    def move_abs_dir(arm_str, abs_dir):
        '''
        Attempts movement of arm_str in abs_dir, and returns whether
        this was successful.

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
            bool
        '''
        new_pose = Link._get_pose_abs_dir(arm_str, abs_dir)
        arm_idx = Link.get_arm_index(arm_str)
        seed = S.arms.arms[arm_idx].get_joint_positions()
        side_joints = Link._get_ik_for_ee_raw(arm_idx, new_pose, seed)

        # If movement isn't possible, return false.
        if side_joints is None:
            return False

        # Else, try movement and returns its success val.
        joints = [None, None]
        joints[arm_idx] = side_joints
        return Link._move_to_joints(joints[0], joints[1])

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
        return Link._move_to_joints(r, l)

    # ##################################################################
    # Private static methods (implementation-specific)
    # ##################################################################

    @staticmethod
    def _get_ik_for_ee_raw(side, wrist_pose, seed):
        '''
        Gets IK for a raw EE pose (i.e. desired Pose determined based on
        known EE pose).

        Args:
            side (int): Side.RIGHT or Side.LEFT
            wrist_pose (Pose): EE-position of wrist link
            seed ([float]): 7-elemnt joint positions

        Returns:
            [float]|None
        '''
        return S.arms.arms[side].get_ik_for_ee(wrist_pose, seed)

    @staticmethod
    def _get_ik_for_ee_computed(side, finger_tip_pose):
        '''
        Gets IK for a computed EE pose (i.e. desired Pose in 3-space
        determined programmatically).

        Args:
            side (int): Side.RIGHT or Side.LEFT
            finger_tip_pose (Pose): EE-position of finger tip

        Returns:
            [float]|None
        '''
        # We are elsewhere computing where we want the finger tips to
        # go; here, we want to compute where we want the wrist link to
        # go, as that is what IK solves for.
        wrist_pose = Link._offset_pose(finger_tip_pose)
        # TODO(mbforbes): Be smarter about seed. We could cache IK
        # results and do a nearest neighbor. Note that we don't have
        # to do any of this if we move to MoveIt!
        seed = [0.0] * 7  # "Default" position (arms forward).
        return Link._get_ik_for_ee_raw(side, wrist_pose, seed)

    @staticmethod
    def _offset_pose(pose, offset=-0.18):
        '''Offsets pose by offset.

        Args:
            pose (Pose): The pose to offset.
            constant (float, optional): How much to offset by. Defaults
                to -0.18 (reverse length of gripper).

        Returns:
            Pose: The offset pose.
        '''
        transform = World.get_matrix_from_pose(pose)
        offset_array = [offset, 0, 0]
        offset_transform = tf.transformations.translation_matrix(offset_array)
        hand_transform = tf.transformations.concatenate_matrices(
            transform, offset_transform)
        return World.get_pose_from_transform(hand_transform)

    @staticmethod
    def _get_pose_abs_dir(arm_str, abs_dir):
        '''
        Returns the pose for a movement of arm_str in abs_dir.

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
            Pose|None: None if EE state can't be found as system not yet
                fully initialized.
        '''
        arm_idx = Link.get_arm_index(arm_str)
        add_vec = Link.abs_dir_map[abs_dir]
        cur_pose = S.arms.arms[arm_idx].get_ee_state()

        # We might have to fail out here if we couldn't get the EE state
        # (this happens e.g. on system startup).
        if cur_pose is None:
            return None

        # Compute the new pose.
        new_pose = Pose(
            Point(
                cur_pose.position.x + add_vec.x,
                cur_pose.position.y + add_vec.y,
                cur_pose.position.z + add_vec.z
            ),
            cur_pose.orientation
        )
        return new_pose

    @staticmethod
    def _move_to_joints(r_joints=None, l_joints=None):
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
