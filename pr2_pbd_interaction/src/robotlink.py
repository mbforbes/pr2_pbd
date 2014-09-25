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

# System builtins
from collections import OrderedDict, defaultdict
import math

# ROS Builtins
from actionlib import SimpleActionClient
from geometry_msgs.msg import Vector3, Pose, Point, Quaternion
import tf

# ROS 3rd party
from pr2_object_manipulation_msgs.msg import (
    IMGUIAction, IMGUIActionGoal, IMGUIActionResult, IMGUIGoal, IMGUIOptions,
    IMGUICommand, IMGUIAdvancedOptions)
from object_manipulation_msgs.msg import ManipulationResult

# PbD
from pr2_pbd_interaction.msg import (
    HandsFreeCommand, Side, ArmState, GripperState)
from pr2_social_gaze.msg import GazeGoal, GazeAction
from arms import Arms
from world import World

# Local (hands-free PbD).
from util import Numbers


# ######################################################################
# Module-level constants
# ######################################################################

# Approximate distance from wrist_roll_link to finger tip (haven't
# actually measured).
GRIPPER_LENGTH = 0.18  # in m, so 0.18 = 18cm (I think)

# How high above table to go? Using ROS packages we wouldn't need to use
# these horrible hacks, but they all don't work, so we're doing this for
# now.
ABOVE_TABLE_Z = 0.08  # in m, so 0.08 = 8cm (I think)

# Maximum time to wait for the head to look at something.
MAX_GAZE_WAIT_TIME = rospy.Duration(2.0)

# How long to wait for a gripper to open/close before measuring. See
# note in set_gripper_state(...) for more details.
GRIPPER_TOGGLE_WAIT_TIME = rospy.Duration(1.5)


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
    gaze_client = None


class Link(object):
    '''This class provides is the interface between HandsFree and the
    rest of the PbD system. It wraps variables and converts types.

    Replacing it and S would make the system PbD-independent.
    '''

    # Orientation options to try for computing IK towards locations. We
    # use an OrderedDict so we know in which order we try them.
    orientations = OrderedDict([
        # NOTE(mbforbes): Trying all upside-down orientations for now as
        # this actually lets the grippers more easily tilt down (the
        # default, "rightside-up" (as I'm calling it) orientation is
        # biased towards tilting upwards, which is less useful for us
        # because the PR2's arms are higher up than the table, and
        # we're only concerned here with tabletop manipulation tasks).
        # ('flat-upwards', Quaternion(
        #     0.0,
        #     0.0,
        #     0.0,
        #     1.0
        # )),

        # 'General' list (mostly pointing fowards or angles of pointing
        # downwards).
        ('gen-flat-upsidedown', Quaternion(
            1.0,
            0.0,
            0.0,
            0.0
        )),
        ('gen-smalltilt-upsidedown', Quaternion(
            0.958600311321,
            0.0389047107548,
            -0.280663429733,
            -0.0282826064416
        )),
        ('gen-45deg-upsidedown', Quaternion(
            0.947183721725,
            0.0378169124572,
            -0.317060828047,
            -0.029754155172
        )),
        ('gen-largetilt-upsidedown', Quaternion(
            0.84375001925,
            0.0296645574088,
            -0.534565233277,
            -0.0380253917916
        )),
        ('gen-vert-upsidedown', Quaternion(
            0.710535569959,
            0.0208582222416,
            -0.702007727379,
            -0.0434659532153
        )),
        ('gen-45deg+righttilt-upsidedown', Quaternion(
            0.816555509917,
            -0.166496173679,
            -0.388895710032,
            0.392780154914
        )),
        ('gen-45deg+right-upsidedown', Quaternion(
            0.626434852697,
            -0.291921804483,
            -0.305953683145,
            0.654792623021
        )),
        ('gen-45deg+lefttilt-upsidedown', Quaternion(
            0.83986672291,
            0.176672428537,
            -0.384052777203,
            -0.34046175272
        )),
        ('gen-45deg+left-upsidedown', Quaternion(
            -0.631060356859,
            -0.316931066071,
            0.279972459512,
            0.650332951091
        )),

        # Pointing downwards
        ('down-1', Quaternion(
            0.70710678118,
            0,
            -0.70710678118,
            0
        )),
        ('down-2', Quaternion(
            -0.5,
            0.5,
            0.5,
            0.5
        )),
        ('down-3', Quaternion(
            0,
            0.70710678118,
            0,
            0.70710678118
        )),
        ('down-4', Quaternion(
            0.5,
            0.5,
            -0.5,
            0.5
        )),

        # Pointing left
        ('left-1', Quaternion(
            0.70710678118,
            0.70710678118,
            0,
            0
        )),
        ('left-2', Quaternion(
            -0.5,
            -0.5,
            0.5,
            0.5
        )),
        ('left-3', Quaternion(
            0,
            0,
            0.70710678118,
            0.70710678118
        )),
        ('left-4', Quaternion(
            0.5,
            0.5,
            0.5,
            0.5
        )),

        # Pointing right
        ('right-1', Quaternion(
            0,
            0,
            -0.70710678118,
            0.70710678118
        )),
        ('right-2', Quaternion(
            0.5,
            -0.5,
            0.5,
            -0.5
        )),
        ('right-3', Quaternion(
            -0.70710678118,
            0.70710678118,
            0,
            0
        )),
        ('right-4', Quaternion(
            -0.5,
            0.5,
            0.5,
            -0.5
        )),

        # Pointing forward
        ('forward-1', Quaternion(
            1,
            0,
            0,
            0
        )),
        ('forward-2', Quaternion(
            -0.70710678118,
            0,
            0,
            0.70710678118
        )),
        ('forward-3', Quaternion(
            0,
            0,
            0,
            1
        )),
        ('forward-4', Quaternion(
            0.70710678118,
            0,
            0,
            0.70710678118
        )),
    ])

    # This maps relative positions (e.g. moving hand "to the left of"
    # something) to the orientation of the hand. If a relative position
    # is not listed, it gets the default entry. Note that this maps to
    # prefixes of the orientation map above, because there are multiple
    # possible "orientations" for a logical orientation.
    #
    # NOTE(mbforbes): Relative positions that have multiple options
    # (e.g. NEXT_TO can be TO_LEFT_OF or TO_RIGHT_OF) don't need to be
    # mapped here because they resolve to one of their concrete options.
    orientation_prefix_map = defaultdict(lambda: 'gen', {
        HandsFreeCommand.ABOVE: 'down',
        HandsFreeCommand.TO_LEFT_OF: 'right',
        HandsFreeCommand.TO_RIGHT_OF: 'left',
        HandsFreeCommand.IN_FRONT_OF: 'forward',
        HandsFreeCommand.BEHIND: 'down',
        HandsFreeCommand.ON_TOP_OF: 'down',
    })

    joint_positions = {
        'to_side': {
            'right': [
                -0.75,  # shoulder_pan
                -0.20,  # shoulder_lift
                -2.60,  # upper_arm_roll
                -0.40,  # elbow_flex
                1.60,  # forearm_roll
                -0.10,  # wrist_flex
                1.00  # wrist_roll
            ],
            'left': [
                0.75,  # shoulder_pan
                -0.20,  # shoulder_lift
                2.60,  # upper_arm_roll
                -0.40,  # elbow_flex
                -1.60,  # forearm_roll
                -0.10,  # wrist_flex
                1.00  # wrist_roll
            ],
        },
    }

    # Settings.
    # How far to move on a 'move' command (if it specifies a direction).
    movement_delta = 0.10  # in m, so 0.10 = 10cm (I think)

    # How far to rotate on a 'rotate' command.
    rotate_delta = 1.6  # I think this is roughly a quarter turn.

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
            S.imgui_action_client.wait_for_server()
        if S.gaze_client is None:
            S.gaze_client = SimpleActionClient(
                'gaze_action',
                GazeAction
            )
            S.gaze_client.wait_for_server()

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
    def clear_objects():
        '''
        Clears objects from backend (PbD).
        '''
        S.world.clear_all_objects()

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
            int: GripperState.OPEN, CLOSED, or HOLDING
        '''
        # NOTE(mbforbes): We can't use PbD's gripper tracking because
        # it doesn't track changes due to pr2_object_manipulation's
        # automatic pick-up.
        joint_val = Link.get_gripper_joint_position(arm_idx)
        arm = 'Right' if arm_idx == Side.RIGHT else 'Left'
        rospy.loginfo(arm + ' gripper joint position: ' + str(joint_val))
        # TODO(mbforbes): Refactor these numbers into constants if they
        # work.
        if joint_val < 0.0025:
            return GripperState.CLOSED
        elif joint_val < 0.078:
            return GripperState.HOLDING
        else:
            return GripperState.OPEN

    @staticmethod
    def set_gripper_state(arm_idx, gripper_state):
        '''
        Sets arm_idx to gripper_state.

        Note that, even when waiting for the action to complete, the arm
        moving action returns too early, leading to incorrect joint
        measurements and reports of 'failure' when success was really
        achieved. (For more info, see _send_gripper_command(...) in
        arm.py and note logging output.) Because of this, we are forced
        to do a bit more waiting here.

        Args:
            arm_idx (int): Side.RIGHT or Side.LEFT
            gripper_state (int): GripperState.OPEN or
                GripperState.CLOSED

        Returns:
            bool: Whether the arm reached the desired gripper_state.
        '''
        if gripper_state == GripperState.OPEN:
            S.arms.arms[arm_idx].open_gripper()
        else:
            S.arms.arms[arm_idx].close_gripper()

        # Pause because the action returns before it's complete.
        rospy.sleep(GRIPPER_TOGGLE_WAIT_TIME)

        # Check success. Opening is only successful if it actually gets
        # open. Closing is successful if it's closed or holding (i.e.
        # anything NOT open).
        post_state = Link.get_gripper_state(arm_idx)
        if gripper_state == GripperState.OPEN:
            return post_state == GripperState.OPEN
        else:
            return post_state != GripperState.OPEN

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
            side,  # arm_selection (0 right, 1 left)
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
    def get_rel_dir_possible(arm_str, pbd_obj, rel_dir):
        '''
        Returns whether a movement of arm_str in rel_dir of pbd_obj is
        possible.

        Args:
            arm_str (str): HandsFreeCommand.LEFT_HAND or
                HandsFreeCommand.RIGHT_HAND
            pbd_obj (PbdObject)
            rel_dir: HandsFreeCommand.AWAY or HandsFreeCommand.TOWARDS

        Returns:
            bool
        '''
        new_pose = Link._get_pose_rel_dir(arm_str, pbd_obj, rel_dir)

        # If we're not initialized yet (early in startup of system),
        # then we must just return false.
        if new_pose is None:
            return False

        arm_idx = Link.get_arm_index(arm_str)
        return Link._get_ik_for_ee_computed(arm_idx, new_pose) is not None

    @staticmethod
    def point_to(pbd_obj, arm_idx):
        '''
        This should not fail if the object exists; if it does,
        implementation needs to be more robust.

        Args:
            pbd_obj (PbdObject): What to point to
            arm_idx (int): Side.RIGHT or Side.LEFT; which arm to use.

        Returns:
            bool: Success?
        '''
        # Calculate desired pose.
        displacement = Vector3(
            Link.BACKWARD_VEC.x + Link.UP_VEC.x,
            Link.BACKWARD_VEC.y + Link.UP_VEC.y,
            Link.BACKWARD_VEC.z + Link.UP_VEC.z,
        )
        obj_pos = pbd_obj.pose.position
        position = Point(
            obj_pos.x + displacement.x,
            obj_pos.y + displacement.y,
            obj_pos.z + displacement.z,
        )
        orientation = Link.orientations['gen-45deg-upsidedown']
        target_fingertip_pose = Pose(position, orientation)

        # Solve IK, go.
        side_joints = Link._get_ik_for_ee_computed(
            arm_idx, target_fingertip_pose)
        if side_joints is None:
            return False
        joints = [None, None]
        joints[arm_idx] = side_joints
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
    def get_cur_orient(arm_idx):
        '''
        Returns the current orientation for the arm_idx arm.

        Args:
            arm_idx (int): Side.RIGHT or Side.LEFT

        Returns:
            Quaternion
        '''
        return S.arms.arms[arm_idx].get_ee_state().orientation

    @staticmethod
    def move_above_table(arm_idx):
        '''
        Moves arm arm_idx to an 'above table' height. Doesn't try to
        move to valid x, y position above table (because we only detect
        part of where the table is, so we don't actually know where it
        ends without looking).

        Args:
            arm_idx (int): Side.RIGHT or Side.LEFT

        Returns:
            bool: Sucess?
        '''
        # Our heuristic.
        if S.world.last_known_table_height is None:
            return False

        # Solve IK.
        wrist_pose = S.arms.arms[arm_idx].get_ee_state()
        finger_tip_pose = Link._offset_pose(wrist_pose, GRIPPER_LENGTH)
        finger_tip_pose.position.z = (
            S.world.last_known_table_height + ABOVE_TABLE_Z)
        side_joints = Link._get_ik_for_ee_computed(arm_idx, finger_tip_pose)
        if side_joints is None:
            return False

        # Move.
        joints = [None, None]
        joints[arm_idx] = side_joints
        return Link._move_to_joints(joints[0], joints[1])

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
    def move_rel_dir(arm_str, pbd_obj, rel_dir):
        '''
        Args:
            arm_str (str): HandsFreeCommand.LEFT_HAND or
                HandsFreeCommand.RIGHT_HAND
            pbd_obj (PbdObject)
            rel_dir: HandsFreeCommand.AWAY or HandsFreeCommand.TOWARDS

        Returns:
            bool: Success?
        '''
        new_pose = Link._get_pose_rel_dir(arm_str, pbd_obj, rel_dir)

        # If we're not initialized yet (early in startup of system),
        # then we must just return false.
        if new_pose is None:
            return False

        arm_idx = Link.get_arm_index(arm_str)
        side_joints = Link._get_ik_for_ee_computed(arm_idx, new_pose)

        # If movement isn't possible, return false.
        if side_joints is None:
            return False

        # Else, try movement and returns its success val.
        joints = [None, None]
        joints[arm_idx] = side_joints
        return Link._move_to_joints(joints[0], joints[1])

    @staticmethod
    def rotate(arm_idx, direction):
        '''
        Rotates final joint in robot's arm_idx arm (wrist roll joint) in
        direction.

        Args:
            arm_idx (int): Side.RIGHT or Side.LEFT
            direction (str): HandsFreeCommand.CW or HandsFreeCommand.CCW

        Returns:
            bool: Success?
        '''
        modifier = 1.0 if direction == HandsFreeCommand.CCW else -1.0
        side_joints = S.arms.arms[arm_idx].get_joint_positions()
        side_joints[-1] = side_joints[-1] + Link.rotate_delta * modifier

        # Try movement and returns its success val.
        joints = [None, None]
        joints[arm_idx] = side_joints
        return Link._move_to_joints(joints[0], joints[1])

    @staticmethod
    def move_to_named_position(name, arm_idx=Side.BOTH):
        '''
        Args:
            name (str): The name of the position to move to. See
                Link.joint_positions.
            arm_idx (int): Side.RIGHT, Side.LEFT, or Side.BOTH

        Returns:
            bool: Whether movement was successful.

        '''
        if name not in Link.joint_positions:
            rospy.logwarn('No pre-set joint positions for ' + str(name))
            return False
        mapping = Link.joint_positions[name]
        r, l = None, None
        if arm_idx == Side.RIGHT or arm_idx == Side.BOTH:
            r = mapping['right']
        if arm_idx == Side.LEFT or arm_idx == Side.BOTH:
            l = mapping['left']
        return Link._move_to_joints(r, l)

    @staticmethod
    def look_at_object(pbd_obj):
        '''
        Args:
            pbd_obj (PbdObject)

        Returns:
            bool: Success?
        '''
        S.gaze_client.send_goal(GazeGoal(
            GazeGoal.LOOK_AT_POINT,
            pbd_obj.pose.position
        ))

        # We just wait for it to finish for consistency with other
        # actions (so we don't return before this is done).
        S.gaze_client.wait_for_result(MAX_GAZE_WAIT_TIME)

        # NOTE(mbforbes): There have been troubles with gaze actions
        # 'never completing', but really being fine. We just always
        # return true.
        return True

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
    def _offset_pose(pose, offset=-GRIPPER_LENGTH):
        '''Offsets pose by offset.

        Args:
            pose (Pose): The pose to offset.
            constant (float, optional): How much to offset by. Defaults
                to -GRIPPER_LENGTH (negative length of gripper).

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
    def _get_pose_rel_dir(arm_str, pbd_obj, rel_dir):
        '''
        Returns the pose for a movement of arm_str in rel_dir of
        pbd_obj.

        Args:
            arm_str (str): HandsFreeCommand.LEFT_HAND or
                HandsFreeCommand.RIGHT_HAND
            pbd_obj (PbdObject)
            rel_dir: HandsFreeCommand.AWAY or HandsFreeCommand.TOWARDS

        Returns:
            Pose|None: None if system isn't fully initialized and can't
                retrieve EE state. Else, Pose in finger-tip (computed)
                space.
        '''
        arm_idx = Link.get_arm_index(arm_str)
        wrist_pose = S.arms.arms[arm_idx].get_ee_state()

        if wrist_pose is None:
            return None
        # Need to offset current pose so that we think about it in
        # "finger tip" and not "gripper" space.
        finger_tip_pose = Link._offset_pose(wrist_pose, GRIPPER_LENGTH)

        # Calculate movement vector
        op = pbd_obj.pose.position
        cp = finger_tip_pose.position
        d_full = Vector3(op.x - cp.x, op.y - cp.y, op.z - cp.z)
        mag = math.sqrt(d_full.x**2 + d_full.y**2 + d_full.z**2)

        # If the difference is up to the movement delta (how far we
        # would normally move), and we're moving towards the object,
        # just move all the way to the object.
        if rel_dir == HandsFreeCommand.TOWARDS and mag <= Link.movement_delta:
            return op

        direction = 1.0 if rel_dir == HandsFreeCommand.TOWARDS else -1.0

        # Otherwise, we scale to only move delta.
        scale = (Link.movement_delta * direction) / mag
        d_scaled = Vector3(
            d_full.x * scale,
            d_full.y * scale,
            d_full.z * scale
        )
        return Pose(
            Point(
                cp.x + d_scaled.x,
                cp.y + d_scaled.y,
                cp.z + d_scaled.z
            ),
            finger_tip_pose.orientation
        )

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
            rospy.loginfo('Side: %d' % (side))
            target = targets[i]
            if target is not None:
                cur_joints = Arms.get_joint_positions(side)
                will_move = False
                # TODO(mbforbes): Debug, this isn't actually working.
                rospy.loginfo('Cur joints:    %s' % (str(cur_joints)))
                rospy.loginfo('Target joints: %s' % (str(target)))
                for j in range(len(cur_joints)):
                    if not Numbers.are_floats_close(target[j], cur_joints[j]):
                        will_move = True
                        rospy.loginfo('Joint idx %d not close enough.' % (j))
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
