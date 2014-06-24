'''Demo.py contains code for building a four-color square-grid block
pattern.'''

__author__ = "mbforbes, priyarao27"

# Must do first
import roslib
roslib.load_manifest('pr2_pbd_interaction')

# ROS
import rospy
from geometry_msgs.msg import Pose, Point, Quaternion, Vector3

# Builtins
import time

# Local
from pr2_pbd_interaction.msg import ArmState, GripperState, Object
from RobotSpeech import RobotSpeech
from Arms import Arms

class Block:
    '''A Block is an instance of a block (or lack thereof). It contains:
    - the block's color (R, G, B, Y, or missing)
    - the x, y, and z coordinates for the placement of the block
    '''
    # Color constants.
    EMPTY = 0
    RED = 1
    BLUE = 2
    GREEN = 3
    YELLOW = 4

    COLOR_STRS = {
        EMPTY: 'EMPTY',
        RED: 'red',
        BLUE: 'blue',
        GREEN: 'green',
        YELLOW: 'yellow',
    }

    def __init__(self, x, y, z, color):
        self.x = x
        self.y = y
        self.z = z
        self.color = color


class Demo:
    '''A Demo is an instance of a block-building task whose basic
    functionality is to:

    - load in a four-color grid of 9 x 9 size
    - convert it to the building format (8 x 8, different color
        constants)
    - iteratively move the hand to the side, request a block, and then
        place the block in the correct position in the grid

    Planned extended functionality should support

    - pausing
    - stopping
    - restarting

    the demo.
    '''

    def __init__(self, arms):
        '''
        Arguments:
         - arms (Arms): The arms of the robot (used in Interaction.py).

        '''
        rospy.loginfo('Constructing new demo')

        # Save vars
        # --------------------------------------------------------------
        self.arms = arms
        self.robotSpeech = RobotSpeech()


        # Location settings
        # --------------------------------------------------------------
        # See image here:
        # https://docs.google.com/document/d/1QxDMV2-GaK5ypKZQn1JguW7REsl2LNNyVvPCX_0pYcE/edit

        # x increases going AWAY from the robot; this is the *farthest*
        # row
        self.xMax = 0.711413491011 # My val. This is min of top l/r.
        # self.xMax = 0.753043424322 # Priya's val
        # xmin (bottom row): 0.354288631421 (max of bottom l/r)
        # gives ~0.044 x / block

        # y increases going LEFT (looking as the robot); this is the
        # *left-most* column (from the robot's perspective)
        self.yMax = -0.0203566341923 # My val
        # self.yMax = -0.0521526380418 # Priya's val
        # ymin (right-most column): -0.437931316074
        # gives ~0.052 y / block

        # How big the blocks are
        self.rawBlocksize = 0.035

        # How much padding to give between blocks. Priya gave 1.5 cm
        # (0.015).
        self.xPadding = 0.005
        self.yPadding = 0.005

        # Final blocksize to use
        self.xBlocksize = self.rawBlocksize + self.xPadding
        self.yBlocksize = self.rawBlocksize + self.yPadding

        # "At block" z position. Originally 0.7.
        self.zBlock = 0.816985346079 # The biggest I saw in measuring.

        # "Above" z position. Originally 0.8. I'm adding 5cm (0.05).
        self.zAbove = self.zBlock + 0.05


        # Robot settings
        # --------------------------------------------------------------
        # NOTE(max): Robot height, measured on PR2 dashboard > joints by
        # torso_lift_joint Position, should be as close to 0.1 as
        # possible.

        # right arm: 0, left arm: 1. Note that the arm cannot be
        # trivially switched as this would change the x, y (and likely
        # z) measurements.
        self.arm_index = 0

        # Pre-set positions
        # This is a pose to the side of the the robot, where it's
        # reaching for a block.
        self.pose_side = Pose(
            Point(0.431058744587, -0.619199050153, 0.967104246181),
            Quaternion(
                0.998904413668,
                0.0279232263886,
                -0.0041013455725,
                -0.0373288728402
            )
        )
        # This is Priya's original side pose.
        # self.pose_side = Pose(
        #     Point(-0.083964934891, -1.00611545184, 0.754503050059),
        #     Quaternion(0.737935404871, -0.673310184246, -0.0442863423926,
        #             -0.0119772244547)
        # )


        # Quaternions (orientations) for the gripper.
        # This orientation is
        self.quat_straight = Quaternion(-0.700782723519, -0.0321300462759,
                0.711998018067, -0.0304968328256)

        # Dummy object used when one needed
        self.dummy_obj = Object(0,'', Pose(Point(),Quaternion()), Vector3())


        # Timing settings
        # --------------------------------------------------------------
        # How long the robot should wait for someone to put a block in
        # its gripper before closing, in seconds.
        self.block_wait_time = 5.0

        # How long to let a block settle before moving the gripper up.
        self.block_settle_time = 2.0


        # Grid settings
        # --------------------------------------------------------------
        # how many blocks we use (8 means 0-7, so 8 vals)
        self.grid_length = 8

        # grid_length^2; total no. blocks
        self.grid_size = self.grid_length ** 2


        # Load 2D color grid
        # --------------------------------------------------------------
        # This will be more elaborate in the future. For now, just
        # hardcode a few blocks for testing.
        #
        # The self.colors is NOT guaranteed to have colors for all
        # locations. It can have empty blocks. However, it's only
        # required to explicitly note colored blocks.
        self.colors = {}
        self.colors[(4,4)] = Block.RED


        # "True" instance variables
        # --------------------------------------------------------------
        # Grid goes
        #
        # (0,0) ---------- + -------------> (0, self.grid_length)
        #   |
        #   |
        #   |
        #   +
        #   |
        #   |
        #   |
        #   v
        # (self.grid_length, 0) ---- + ---> (self.grid_length, self.grid_length)
        #
        # The grid is guaranteed to have blocks in all valid grid
        # locations; if no block is present, it will be represented by
        # a Block object with the color field set to Block.EMPTY.
        self.grid = {}
        for row in range (self.grid_length):
            for col in range(self.grid_length):
                x = self.xMax - row * self.xBlocksize
                y = self.yMax - col * self.yBlocksize
                z = self.zBlock
                color = (self.colors[(row,col)] if
                        self.colors.has_key((row,col)) else Block.EMPTY)
                self.grid[(row,col)] = Block(x, y, z, color)


        # Control variables
        # --------------------------------------------------------------
        # This will be set to True when the demo starts; can set to
        # False in order to get the demo to stop.
        self.running = False

        # This is set when the Demo believes that it has finished
        # placing (or checking empty) all block locations.
        self.complete = False

        # These are the next coordinates to place (or check if the
        # location is empty). These are only incremented after a full
        # place operation is completed.
        self.nextX = 0
        self.nextY = 0

    def next_block(self):
        '''This moves the block variables to the next block on the grid.

        It returns true if there is a valid next block, or false if we
        were already at the end of the grid. It DOES NOT reset the
        varialbes to the beginning of the grid.'''
        if self.nextY < self.grid_length - 1:
            # Just move the Y one (to the right).
            self.nextY += 1
            return True
        else:
            # Y is at the end. See if we have another row to go to.
            if self.nextX < self.grid_length - 1:
                # Just move the X one (down), and reset y to 0.
                self.nextX += 1
                self.nextY = 0
                return True
            else:
                # We are at the end of the grid. Move nothing.
                return False

    def stop(self):
        '''This doesn't stop the running of the demo directly; it sets a
        flag such that the run method will pause after finishing the
        block it's currently working on (either checking for emptyness
        or actually placing).'''
        self.running = False

    def run(self):
        '''This starts the demo running at the current self.nextX and
        self.nextY.'''
        rospy.loginfo('Running demo.')
        self.running = True
        while self.running and not self.complete:
            # Do empty check
            if self.grid[(self.nextX, self.nextY)].color != Block.EMPTY:
                # Block is colored; get and place it.
                self.getBlock()
                colorstr = Block.COLOR_STRS[
                        self.grid[(self.nextX, self.nextY)].color]
                rospy.loginfo('Placing ' + colorstr + ' block at (' +
                        str(self.nextX) + ', ' + str(self.nextY) + ').')
                self.placeBlock()

            # Regardless of colored or empty, move to next.
            if not self.next_block():
                # If there was no next block, we mark the demo as
                # complete and stop running.
                rospy.loginfo('Demo complete.')
                self.complete = True
                self.running = False

    def reset(self):
        '''This resets the next block marker to the first block (which
        is (0,0)) and sets it as not completed. It does not start or
        stop the demo. The demo must be stopped for this to work.'''
        if self.running:
            # Bad! Demo must be stopped.
            rospy.logwarn('Demo must be stopped to reset.')
        else:
            # Demo is stopped; reset.
            rospy.loginfo('Resetting demo.')
            self.nextX = 0
            self.nextY = 0
            self.complete = False

    def sayColor(self):
        '''Says the block color of the block in (nextX, nextY).'''
        color = self.grid[(self.nextX, self.nextY)].color
        sayStr = Block.COLOR_STRS[color] + ' block'
        rospy.loginfo('Robot wants ' + sayStr)
        self.robotSpeech.say(sayStr)

    def waitForBlock(self):
        '''Waits for user to put a block in the gripper.

        Simple implementation is just a fixed time sleep. Could do
        something more complicated in the future with signaling to speed
        up.'''
        time.sleep(self.block_wait_time)

    def getBlock(self):
        '''Moves arm to the side and asks for the block color for the
        block in self.nextX and self.nextY.

        TODO(max): Should the opening here be partial or all the way?'''
        # Move arm to the side
        rospy.loginfo('Moving arm to side to get block.')
        self.moveToPose(self.pose_side)

        # Open gripper
        self.arms.set_gripper_state(self.arm_index, GripperState.OPEN,
                wait=False)

        # Request block
        self.sayColor()

        # Wait for block
        self.waitForBlock()

        # Close gripper
        self.arms.set_gripper_state(self.arm_index, GripperState.CLOSED,
                wait=True)

    def moveToPose(self, target_pose):
        '''Moves the robot arm to a predefined pose.

        Arguments:
         - target_pose (Pose): Pose to move to, relative to robot base.
         '''
        ik_solution = self.get_ik(target_pose)
        if ik_solution is not None:
            arm_joints = [None, None]
            arm_joints[self.arm_index] = ik_solution
            success = self.arms.move_to_joints(arm_joints[0], arm_joints[1])
            if not success:
                p = target_pose.position
                rospy.logerr('Move to (' + str(p.x) + ', ' + str(p.y) +
                        ', ' + str(p.z) + ') was obstructed!')
        else:
            # Extract pose's position's x,y,z for better error
            # reporting.
            p = target_pose.position
            rospy.logerr('Could not move to (' + str(p.x) + ', ' + str(p.y) +
                    ', ' + str(p.z) + ')!')

    def moveToPosition(self, x, y, z):
        '''Moves the robot arm to a predefined position, using the
        default arm orientation (self.quat_straight).

        Arguments:
         - x (float): Point in robot space (m), relative to robot base.
         - y (float): Point in robot space (m), relative to robot base.
         - z (float): Point in robot space (m), relative to robot base.
         '''
        target_pose = Pose(Point(x, y, z), self.quat_straight)
        self.moveToPose(target_pose)

    def placeBlock(self):
        '''After the robot has a block, this will place the block on the
        table.'''
        block = self.grid[(self.nextX, self.nextY)]

        # Move above the block's intended position
        rospy.loginfo('Moving arm above.')
        self.moveToPosition(block.x, block.y, self.zAbove)

        # Move to the block's intended position
        rospy.loginfo('Moving arm down.')
        self.moveToPosition(block.x, block.y, block.z)

        # Open the gripper to release the block
        self.arms.set_gripper_state(self.arm_index, GripperState.OPEN,
                wait=True)

        # Wait a bit to let the block settle
        # TODO(max): Investigate this so we can hopefully speed up.
        #time.sleep(self.block_settle_time)

        # Move the arm back up to the "above" position.
        rospy.loginfo('Moving arm above (again).')
        self.moveToPosition(block.x, block.y, self.zAbove)

    def get_ik(self, target_pose):
        '''Tries the IK solver to get an IK solution.

        Arguments:
         - target_pose (Pose): The target ee pose.

        Returns:
         - arm_state | None: Either the arm state, including, the solved
                 joint angles, or None if no solution was found.
        '''
        arm_state = ArmState(
            ArmState.ROBOT_BASE,
            target_pose,
            # "default" (all 0s) seed is actually pretty good for us
            # as that is the arm straight out.
            [0., 0., 0., 0., 0., 0., 0.],
            self.dummy_obj
        )
        ik_solution, has_solution = Arms.solve_ik_for_arm(self.arm_index,
                arm_state)
        if has_solution:
            # Hooray! MoveIt! failed but PbD IK solver succeeded.
            return ik_solution
        else:
            rospy.logerr('IK solver found no solution.')
            return None
