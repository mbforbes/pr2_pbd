'''Main interaction loop'''

import roslib
roslib.load_manifest('pr2_pbd_interaction')

# Generic libraries
import rospy
import time
from visualization_msgs.msg import MarkerArray

# Local stuff
from World import World
from RobotSpeech import RobotSpeech
from Session import Session
from Response import Response
from Arms import Arms
from Arm import ArmMode
from pr2_pbd_interaction.msg import ArmState, GripperState
from pr2_pbd_interaction.msg import ActionStep, ArmTarget, Object
from pr2_pbd_interaction.msg import GripperAction, ArmTrajectory
from pr2_pbd_interaction.msg import ExecutionStatus, GuiCommand
from pr2_pbd_speech_recognition.msg import Command
from pr2_social_gaze.msg import GazeGoal
from geometry_msgs.msg import Pose, Point, Quaternion, Vector3

from ik import IK

class Interaction:
    '''Finite state machine for the human interaction'''

    _is_programming = True
    _is_recording_motion = False
    _arm_trajectory = None
    _trajectory_start_time = None

    def __init__(self):
        self.arms = Arms()
        self.world = World()
        self.session = Session(object_list=self.world.get_frame_list(),
                               is_debug=True)
        self._viz_publisher = rospy.Publisher('visualization_marker_array',
                                              MarkerArray)

        rospy.Subscriber('recognized_command', Command, self.speech_command_cb)
        rospy.Subscriber('gui_command', GuiCommand, self.gui_command_cb)

        self._undo_function = None

        self.responses = {
            Command.TEST_MICROPHONE: Response(Interaction.empty_response,
                                [RobotSpeech.TEST_RESPONSE, GazeGoal.NOD]),
            Command.RELAX_RIGHT_ARM: Response(self.relax_arm, 0),
            Command.RELAX_LEFT_ARM: Response(self.relax_arm, 1),
            Command.OPEN_RIGHT_HAND: Response(self.open_hand, 0),
            Command.OPEN_LEFT_HAND: Response(self.open_hand, 1),
            Command.CLOSE_RIGHT_HAND: Response(self.close_hand, 0),
            Command.CLOSE_LEFT_HAND: Response(self.close_hand, 1),
            Command.STOP_EXECUTION: Response(self.stop_execution, None),
            Command.UNDO: Response(self.undo, None),
            Command.DELETE_ALL_STEPS: Response(self.delete_all_steps, None),
            Command.DELETE_LAST_STEP: Response(self.delete_last_step, None),
            Command.FREEZE_RIGHT_ARM: Response(self.freeze_arm, 0),
            Command.FREEZE_LEFT_ARM: Response(self.freeze_arm, 1),
            Command.CREATE_NEW_ACTION: Response(self.create_action, None),
            Command.EXECUTE_ACTION: Response(self.execute_action, None),
            Command.NEXT_ACTION: Response(self.next_action, None),
            Command.PREV_ACTION: Response(self.previous_action, None),
            Command.SAVE_POSE: Response(self.save_step, None),
            Command.RECORD_OBJECT_POSE: Response(
                                            self.record_object_pose, None),
            Command.START_RECORDING_MOTION: Response(
                                            self.start_recording, None),
            Command.STOP_RECORDING_MOTION: Response(self.stop_recording, None)
            }
        self.ik_left = IK("l")
        self.ik_right = IK("r")
        rospy.loginfo('Interaction initialized.')

    def open_hand(self, arm_index):
        '''Opens gripper on the indicated side'''
        if self.arms.set_gripper_state(arm_index, GripperState.OPEN):
            speech_response = Response.open_responses[arm_index]
            if (Interaction._is_programming and self.session.n_actions() > 0):
                self.save_gripper_step(arm_index, GripperState.OPEN)
                speech_response = (speech_response + ' ' +
                                   RobotSpeech.STEP_RECORDED)
            return [speech_response, Response.glance_actions[arm_index]]
        else:
            return [Response.already_open_responses[arm_index],
                    Response.glance_actions[arm_index]]

    def close_hand(self, arm_index):
        '''Closes gripper on the indicated side'''
        if Arms.set_gripper_state(arm_index, GripperState.CLOSED):
            speech_response = Response.close_responses[arm_index]
            if (Interaction._is_programming and self.session.n_actions() > 0):
                self.save_gripper_step(arm_index, GripperState.CLOSED)
                speech_response = (speech_response + ' ' +
                                   RobotSpeech.STEP_RECORDED)
            return [speech_response, Response.glance_actions[arm_index]]
        else:
            return [Response.already_closed_responses[arm_index],
                    Response.glance_actions[arm_index]]

    def relax_arm(self, arm_index):
        '''Relaxes arm on the indicated side'''
        if self.arms.set_arm_mode(arm_index, ArmMode.RELEASE):
            return [Response.release_responses[arm_index],
                    Response.glance_actions[arm_index]]
        else:
            return [Response.already_released_responses[arm_index],
                    Response.glance_actions[arm_index]]

    def freeze_arm(self, arm_index):
        '''Stiffens arm on the indicated side'''
        if self.arms.set_arm_mode(arm_index, ArmMode.HOLD):
            return [Response.hold_responses[arm_index],
                    Response.glance_actions[arm_index]]
        else:
            return [Response.already_holding_responses[arm_index],
                    Response.glance_actions[arm_index]]

    def edit_action(self, dummy=None):
        '''Goes back to edit mode'''
        if (self.session.n_actions() > 0):
            if (Interaction._is_programming):
                return [RobotSpeech.ALREADY_EDITING, GazeGoal.SHAKE]
            else:
                Interaction._is_programming = True
                return [RobotSpeech.SWITCH_TO_EDIT_MODE, GazeGoal.NOD]
        else:
            return [RobotSpeech.ERROR_NO_SKILLS, GazeGoal.SHAKE]

    def save_action(self, dummy=None):
        '''Goes out of edit mode'''
        self.session.save_current_action()
        Interaction._is_programming = False
        return [RobotSpeech.ACTION_SAVED + ' ' +
                str(self.session.current_action_index), GazeGoal.NOD]

    def create_action(self, dummy=None):
        '''Creates a new empty action'''
        self.world.clear_all_objects()
        self.session.new_action()
        Interaction._is_programming = True
        return [RobotSpeech.SKILL_CREATED + ' ' +
                str(self.session.current_action_index), GazeGoal.NOD]

    def next_action(self, dummy=None):
        '''Switches to next action'''
        if (self.session.n_actions() > 0):
            if self.session.next_action(self.world.get_frame_list()):
                return [RobotSpeech.SWITCH_SKILL + ' ' +
                        str(self.session.current_action_index), GazeGoal.NOD]
            else:
                return [RobotSpeech.ERROR_NEXT_SKILL + ' ' +
                        str(self.session.current_action_index), GazeGoal.SHAKE]
        else:
            return [RobotSpeech.ERROR_NO_SKILLS, GazeGoal.SHAKE]

    def previous_action(self, dummy=None):
        '''Switches to previous action'''
        if (self.session.n_actions() > 0):
            if self.session.previous_action(self.world.get_frame_list()):
                return [RobotSpeech.SWITCH_SKILL + ' ' +
                        str(self.session.current_action_index), GazeGoal.NOD]
            else:
                return [RobotSpeech.ERROR_PREV_SKILL + ' ' +
                        str(self.session.current_action_index), GazeGoal.SHAKE]
        else:
            return [RobotSpeech.ERROR_NO_SKILLS, GazeGoal.SHAKE]

    def delete_last_step(self, dummy=None):
        '''Deletes last step of the current action'''
        if (self.session.n_actions() > 0):
            if (Interaction._is_programming):
                if self.session.n_frames() > 0:
                    self.session.delete_last_step()
                    self._undo_function = self._resume_last_step
                    return [RobotSpeech.LAST_POSE_DELETED, GazeGoal.NOD]
                else:
                    return [RobotSpeech.SKILL_EMPTY, GazeGoal.SHAKE]
            else:
                return ['Action ' + str(self.session.current_action_index) +
                        RobotSpeech.ERROR_NOT_IN_EDIT, GazeGoal.SHAKE]
        else:
            return [RobotSpeech.ERROR_NO_SKILLS, GazeGoal.SHAKE]

    def delete_all_steps(self, dummy=None):
        '''Deletes all steps in the current action'''
        if (self.session.n_actions() > 0):
            if (Interaction._is_programming):
                if self.session.n_frames() > 0:
                    self.session.clear_current_action()
                    self._undo_function = self._resume_all_steps
                    return [RobotSpeech.SKILL_CLEARED, GazeGoal.NOD]
                else:
                    return [RobotSpeech.SKILL_EMPTY, None]
            else:
                return ['Action ' + str(self.session.current_action_index) +
                        RobotSpeech.ERROR_NOT_IN_EDIT, GazeGoal.SHAKE]
        else:
            return [RobotSpeech.ERROR_NO_SKILLS, GazeGoal.SHAKE]

    def undo(self, dummy=None):
        '''Undoes the effect of the previous command'''
        if (self._undo_function == None):
            return [RobotSpeech.ERROR_NOTHING_TO_UNDO, GazeGoal.SHAKE]
        else:
            return self._undo_function()

    def _resume_all_steps(self):
        '''Resumes all steps after clearing'''
        self.session.undo_clear()
        return [RobotSpeech.ALL_POSES_RESUMED, GazeGoal.NOD]

    def _resume_last_step(self):
        '''Resumes last step after deleting'''
        self.session.resume_deleted_step()
        return [RobotSpeech.POSE_RESUMED, GazeGoal.NOD]

    def stop_execution(self, dummy=None):
        '''Stops ongoing execution'''
        if (self.arms.is_executing()):
            self.arms.stop_execution()
            return [RobotSpeech.STOPPING_EXECUTION, GazeGoal.NOD]
        else:
            return [RobotSpeech.ERROR_NO_EXECUTION, GazeGoal.SHAKE]

    def save_gripper_step(self, arm_index, gripper_state):
        '''Saves an action step that involves a gripper state change'''
        if (self.session.n_actions() > 0):
            if (Interaction._is_programming):
                states = self._get_arm_states()
                step = ActionStep()
                step.type = ActionStep.ARM_TARGET
                step.armTarget = ArmTarget(states[0], states[1], 0.2, 0.2)
                actions = [self.arms.get_gripper_state(0),
                           self.arms.get_gripper_state(1)]
                actions[arm_index] = gripper_state
                step.gripperAction = GripperAction(actions[0], actions[1])
                self.session.add_step_to_action(step,
                                                self.world.get_frame_list())

    def start_recording(self, dummy=None):
        '''Starts recording continuous motion'''
        if (self.session.n_actions() > 0):
            if (Interaction._is_programming):
                if (not Interaction._is_recording_motion):
                    Interaction._is_recording_motion = True
                    Interaction._arm_trajectory = ArmTrajectory()
                    Interaction._trajectory_start_time = rospy.Time.now()
                    return [RobotSpeech.STARTED_RECORDING_MOTION,
                            GazeGoal.NOD]
                else:
                    return [RobotSpeech.ALREADY_RECORDING_MOTION,
                            GazeGoal.SHAKE]
            else:
                return ['Action ' + str(self.session.current_action_index) +
                        RobotSpeech.ERROR_NOT_IN_EDIT, GazeGoal.SHAKE]
        else:
            return [RobotSpeech.ERROR_NO_SKILLS, GazeGoal.SHAKE]

    def stop_recording(self, dummy=None):
        '''Stops recording continuous motion'''
        if (Interaction._is_recording_motion):
            Interaction._is_recording_motion = False
            traj_step = ActionStep()
            traj_step.type = ActionStep.ARM_TRAJECTORY

            waited_time = Interaction._arm_trajectory.timing[0]
            for i in range(len(Interaction._arm_trajectory.timing)):
                Interaction._arm_trajectory.timing[i] -= waited_time
                Interaction._arm_trajectory.timing[i] += rospy.Duration(0.1)

            self._fix_trajectory_ref()
            traj_step.arm_trajectory = ArmTrajectory(
                Interaction._arm_trajectory.rArm[:],
                Interaction._arm_trajectory.lArm[:],
                Interaction._arm_trajectory.timing[:],
                Interaction._arm_trajectory.r_ref,
                Interaction._arm_trajectory.l_ref,
                Interaction._arm_trajectory.r_ref_name,
                Interaction._arm_trajectory.l_ref_name)
            traj_step.gripperAction = GripperAction(
                                        self.arms.get_gripper_state(0),
                                        self.arms.get_gripper_state(1))
            self.session.add_step_to_action(traj_step,
                                        self.world.get_frame_list())
            Interaction._arm_trajectory = None
            Interaction._trajectory_start_time = None
            return [RobotSpeech.STOPPED_RECORDING_MOTION + ' ' +
                    RobotSpeech.STEP_RECORDED, GazeGoal.NOD]
        else:
            return [RobotSpeech.MOTION_NOT_RECORDING, GazeGoal.SHAKE]

    def _fix_trajectory_ref(self):
        '''Makes the reference frame of continuous trajectories uniform'''
        r_ref, r_ref_name = self._find_dominant_ref(
                                        Interaction._arm_trajectory.rArm)
        l_ref, l_ref_name = self._find_dominant_ref(
                                        Interaction._arm_trajectory.lArm)
        for i in range(len(Interaction._arm_trajectory.timing)):
            Interaction._arm_trajectory.rArm[i] = World.convert_ref_frame(
                            Interaction._arm_trajectory.rArm[i],
                            r_ref, r_ref_name)
            Interaction._arm_trajectory.lArm[i] = World.convert_ref_frame(
                            Interaction._arm_trajectory.lArm[i],
                            l_ref, l_ref_name)
        Interaction._arm_trajectory.r_ref = r_ref
        Interaction._arm_trajectory.l_ref = l_ref
        Interaction._arm_trajectory.r_ref_name = r_ref_name
        Interaction._arm_trajectory.l_ref_name = l_ref_name

    def _find_dominant_ref(self, arm_traj):
        '''Finds the most dominant reference frame
        in a continuous trajectory'''
        ref_names = self.world.get_frame_list()
        ref_counts = dict()
        for i in range(len(ref_names)):
            ref_counts[ref_names[i]] = 0
        for i in range(len(arm_traj)):
            if (arm_traj[i].refFrameName in ref_counts.keys()):
                ref_counts[arm_traj[i].refFrameName] += 1
            else:
                rospy.logwarn('Ignoring object with reference frame name '
                    + arm_traj[i].refFrameName
                    + ' because the world does not have this object.')
        dominant_ref = ref_counts.values().index(
                                                max(ref_counts.values()))
        dominant_ref_name = ref_counts.keys()[dominant_ref]
        return World.get_ref_from_name(dominant_ref_name), dominant_ref_name

    def _save_arm_to_trajectory(self):
        '''Saves current arm state into continuous trajectory'''
        if (Interaction._arm_trajectory != None):
            states = self._get_arm_states()
            Interaction._arm_trajectory.rArm.append(states[0])
            Interaction._arm_trajectory.lArm.append(states[1])
            Interaction._arm_trajectory.timing.append(
                        rospy.Time.now() - Interaction._trajectory_start_time)

    def save_step(self, dummy=None):
        '''Saves current arm state as an action step'''
        if (self.session.n_actions() > 0):
            if (Interaction._is_programming):
                states = self._get_arm_states()
                step = ActionStep()
                step.type = ActionStep.ARM_TARGET
                step.armTarget = ArmTarget(states[0], states[1],
                                           0.2, 0.2)
                step.gripperAction = GripperAction(
                                            self.arms.get_gripper_state(0),
                                            self.arms.get_gripper_state(1))
                self.session.add_step_to_action(step,
                                            self.world.get_frame_list())
                return [RobotSpeech.STEP_RECORDED, GazeGoal.NOD]
            else:
                return ['Action ' + str(self.session.current_action_index) +
                        RobotSpeech.ERROR_NOT_IN_EDIT, GazeGoal.SHAKE]
        else:
            return [RobotSpeech.ERROR_NO_SKILLS, GazeGoal.SHAKE]

    def _get_arm_states(self):
        '''Returns the current arms states in the right format'''
        abs_ee_poses = [Arms.get_ee_state(0),
                      Arms.get_ee_state(1)]
        joint_poses = [Arms.get_joint_state(0),
                      Arms.get_joint_state(1)]

        rel_ee_poses = [None, None]
        states = [None, None]

        for arm_index in [0, 1]:
            if (not World.has_objects()):
                # Absolute
                states[arm_index] = ArmState(ArmState.ROBOT_BASE,
                    abs_ee_poses[arm_index], joint_poses[arm_index], Object())
            else:
                nearest_obj = self.world.get_nearest_object(
                                                    abs_ee_poses[arm_index])

                if (nearest_obj == None):
                    states[arm_index] = ArmState(ArmState.ROBOT_BASE,
                                        abs_ee_poses[arm_index],
                                        joint_poses[arm_index], Object())
                else:
                    # Relative
		    #rospy.loginfo('Pose is relative for arm ' + str(arm_index))
                    rel_ee_poses[arm_index] = World.transform(
                                        abs_ee_poses[arm_index],
                                        'base_link', nearest_obj.name)
                    states[arm_index] = ArmState(ArmState.OBJECT,
                                        rel_ee_poses[arm_index],
                                        joint_poses[arm_index], nearest_obj)
        return states

    def execute_action(self, dummy=None):
        '''Starts the execution of the current action'''
	execution_z_offset = -0.00
        if (self.session.n_actions() > 0):
            if (self.session.n_frames() > 1):
                self.session.save_current_action()
                action = self.session.get_current_action()

                if (action.is_object_required()):
                    if (self.world.update_object_pose()):
                        self.session.get_current_action().update_objects(
                                                self.world.get_frame_list())
                        self.arms.start_execution(action, execution_z_offset)
                    else:
                        return [RobotSpeech.OBJECT_NOT_DETECTED,
                                GazeGoal.SHAKE]
                else:
                    self.arms.start_execution(action, execution_z_offset)

                return [RobotSpeech.START_EXECUTION + ' ' +
                        str(self.session.current_action_index), None]
            else:
                return [RobotSpeech.EXECUTION_ERROR_NOPOSES + ' ' +
                        str(self.session.current_action_index), GazeGoal.SHAKE]
        else:
            return [RobotSpeech.ERROR_NO_SKILLS, GazeGoal.SHAKE]

    def speech_command_cb(self, command):
        '''Callback for when a speech command is received'''
        if command.command in self.responses.keys():
            rospy.loginfo('\033[32m Calling response for command ' +
                          command.command + '\033[0m')
            response = self.responses[command.command]

            if (not self.arms.is_executing()):
                if (self._undo_function != None):
                    response.respond()
                    self._undo_function = None
                else:
                    response.respond()
            else:
                if command.command == Command.STOP_EXECUTION:
                    response.respond()
                else:
                    rospy.logwarn('Ignoring speech command during execution: '
                                  + command.command)
        else:
            switch_command = 'SWITCH_TO_ACTION'
            if (switch_command in command.command):
                action_no = command.command[
                                len(switch_command):len(command.command)]
                action_no = int(action_no)
                if (self.session.n_actions() > 0):
                    self.session.switch_to_action(action_no,
                                                  self.world.get_frame_list())
                    response = Response(Interaction.empty_response,
                        [RobotSpeech.SWITCH_SKILL + str(action_no),
                         GazeGoal.NOD])
                else:
                    response = Response(Interaction.empty_response,
                        [RobotSpeech.ERROR_NO_SKILLS, GazeGoal.SHAKE])
                response.respond()
            else:
                rospy.logwarn('\033[32m This command (' + command.command
                              + ') is unknown. \033[0m')

    def gui_command_cb(self, command):
        '''Callback for when a GUI command is received'''

        if (not self.arms.is_executing()):
            if (self.session.n_actions() > 0):
                if (command.command == GuiCommand.SWITCH_TO_ACTION):
                    action_no = command.param
                    self.session.switch_to_action(action_no,
                                                  self.world.get_frame_list())
                    response = Response(Interaction.empty_response,
                        [RobotSpeech.SWITCH_SKILL + str(action_no),
                         GazeGoal.NOD])
                    response.respond()
                elif (command.command == GuiCommand.SELECT_ACTION_STEP):
                    step_no = command.param
                    self.session.select_action_step(step_no)
                    rospy.loginfo('Selected action step ' + str(step_no))
                else:
                    rospy.logwarn('\033[32m This command (' + command.command
                                  + ') is unknown. \033[0m')
            else:
                response = Response(Interaction.empty_response,
                    [RobotSpeech.ERROR_NO_SKILLS, GazeGoal.SHAKE])
                response.respond()
        else:
            rospy.logwarn('Ignoring GUI command during execution: ' +
                                command.command)

    def update(self):
        '''General update for the main loop'''
        self.arms.update()

        if (self.arms.status != ExecutionStatus.NOT_EXECUTING):
            if (self.arms.status != ExecutionStatus.EXECUTING):
                self._end_execution()
        if (Interaction._is_recording_motion):
            self._save_arm_to_trajectory()

        is_world_changed = self.world.update()
        if (self.session.n_actions() > 0):
            action = self.session.get_current_action()
            action.update_viz()
            r_target = action.get_requested_targets(0)
            if (r_target != None):
                self.arms.start_move_to_pose(r_target, 0)
                action.reset_targets(0)
            l_target = action.get_requested_targets(1)
            if (l_target != None):
                self.arms.start_move_to_pose(l_target, 1)
                action.reset_targets(1)

            action.delete_requested_steps()

            states = self._get_arm_states()
            action.change_requested_steps(states[0], states[1])

            if (is_world_changed):
                rospy.loginfo('The world has changed.')
                self.session.get_current_action().update_objects(
                                        self.world.get_frame_list())

        time.sleep(0.1)

    def _end_execution(self):
        '''Responses for when the action execution ends'''
        if (self.arms.status == ExecutionStatus.SUCCEEDED):
            Response.say(RobotSpeech.EXECUTION_ENDED)
            Response.perform_gaze_action(GazeGoal.NOD)
        elif (self.arms.status == ExecutionStatus.PREEMPTED):
            Response.say(RobotSpeech.EXECUTION_PREEMPTED)
            Response.perform_gaze_action(GazeGoal.SHAKE)
        else:
            Response.say(RobotSpeech.EXECUTION_ERROR_NOIK)
            Response.perform_gaze_action(GazeGoal.SHAKE)

        self.arms.status = ExecutionStatus.NOT_EXECUTING

    def record_object_pose(self, dummy=None):
        '''Makes the robot look for a table and objects'''
        objs = self.world.get_frame_list()
        # Settings
        arm_index = 0 # right arm: 0, left arm: 1
        # Open hand
        self.open_hand(arm_index)

# Original
# Quaternion(-0.69987504269, -0.0289561889715, 0.712701230833, -0.0373285320999)),

#POS1

#  position: 
#    x: 0.725304016651
#    y: 0.067733625044
#    z: 0.772047444606
#  orientation: 
#    x: -0.700782723519
#    y: -0.0321300462759
#    z: 0.711998018067
#    w: -0.0304968328256
#joint_pose: [ 0.23665961 -0.03363956 -1.6177423  -0.20996282 -1.50953665 -1.60760814
#  0.35809484]
# 1: [ 0.23659336 -0.02729438 -1.55       -0.20994583 -1.57710341 -1.61551437
#  0.35831203]

        pose_new = Pose(Point(0.725304016651, 0.067733625044,
                             0.772047444606),
                        Quaternion(-0.700782723519, -0.0321300462759,
                             0.711998018067, -0.0304968328256))
        ik_solution = self.ik_right.get_ik_for_ee(pose_new) 

        rospy.logwarn('')
        rospy.logwarn('Joint Pose:')
        rospy.logwarn(str(ik_solution))


        rospy.loginfo('Position 1.')

        arm_state = ArmState(
            0,
            Pose(
                Point(0.725304016651, 0.067733625044,
                    0.772047444606),
                Quaternion(-0.700782723519, -0.0321300462759,
                    0.711998018067, -0.0304968328256)
            ),
            [0.23659336, -0.02729438, -1.55, -0.20994583, -1.57710341, -1.61551437, 0.35831203],
            Object(0,'', Pose(Point(),Quaternion()), Vector3())
        )
        self.arms.start_move_to_pose(arm_state, arm_index)

#POS2

#  position: 
#    x: 0.729687995221
#    y: -0.437537436015
#    z: 0.765807557296
#  orientation: 
#    x: -0.700998025188
#    y: -0.0308937799918
#    z: 0.711850579084
#    w: -0.0302670794436
#joint_pose: [-0.36589188 -0.04089563 -1.88060239 -0.15083799 -1.26160595 -1.58048485
# -0.3088148 ]

#        rospy.loginfo('Position 2.')
#        arm_state = ArmState(
#           0,
#            Pose(
#                Point(0.729687995221, -0.437537436015,
#                    0.765807557296),
#                Quaternion(-0.700998025188, -0.0308937799918,
#                    0.711850579084, -0.0302670794436)
#            ),
#            [-0.36589188, -0.04089563, -1.88060239, -0.15083799, -1.26160595, -1.58048485, -0.3088148],
#            Object(0,'', Pose(Point(),Quaternion()), Vector3())
#        )
#        self.arms.start_move_to_pose(arm_state, arm_index)

#POS3

#  position: 
#    x: 0.767395151335
#    y: -0.188000575716
#    z: 0.79914341326
#  orientation: 
#    x: -2.08137043025e-07
#    y: -0.124679586268
#    z: -3.41342162025e-07
#    w: 0.992197057427
#joint_pose: [-0.12986223 -0.03705663 -1.54889209 -1.64781875 -1.51623231 -1.58902625
#  1.43039164]


#        rospy.loginfo('Position 3.')
#        arm_state = ArmState(
#            0,
#            Pose(
#                Point(0.767395151335, -0.188000575716,
#                    0.79914341326),
#                Quaternion(-2.08137043025e-07, -0.124679586268,
#                   -3.41342162025e-07, 0.992197057427)
#           ),
#           [-0.12986223, -0.03705663, -1.54889209, -1.64781875, -1.51623231, -1.58902625,  1.43039164],
#           Object(0,'', Pose(Point(),Quaternion()), Vector3())
#        )
#        self.arms.start_move_to_pose(arm_state, arm_index)

#POS4

#  position: 
#    x: 0.725304016651
#    y: 0.067733625044
#    z: 0.772047444606
#  orientation: 
#    x: -0.700782723519
#    y: -0.0321300462759
#    z: 0.711998018067
#    w: -0.0304968328256
#joint_pose: [ 0.23665961 -0.03363956 -1.6177423  -0.20996282 -1.50953665 -1.60760814
#  0.35809484]

#        rospy.loginfo('Position 4.')
#        arm_state = ArmState(
#            0,
#            Pose(
#                Point(0.725304016651, -0.067733625044,
#                    1.772047444606),
#                Quaternion(-0.69987504269, -0.0289561889715,
#                    0.712701230833, -0.0373285320999)
#            ),
#            [0.23665961, -0.03363956, -1.6177423,  -0.20996282, -1.50953665, -1.60760814, 0.35809484],
#            Object(0,'', Pose(Point(),Quaternion()), Vector3())
#        )
#        self.arms.start_move_to_pose(arm_state, arm_index)



#Default Pose

#  position: 
#    x: -0.083964934891
#    y: -1.00611545184
#    z: 0.754503050059
#  orientation: 
#    x: 0.737935404871
#    y: -0.673310184246
#    z: -0.0442863423926
#    w: -0.0119772244547
#joint_pose: [-1.66652764  0.02079164 -1.18075147 -0.15008017  4.70036589 -0.12677397
# -0.42011605]

#        rospy.loginfo('Default Position')
#        arm_state = ArmState(
#            0,
#            Pose(
#                Point(-0.083964934891, -1.00611545184,
#                    0.754503050059),
#                Quaternion(0.737935404871, -0.673310184246,
#                    -0.0442863423926, -0.0119772244547)
#            ),
#            [-1.66652764, 0.02079164, -1.18075147, -0.15008017, 4.70036589, -0.12677397, -0.42011605],
#            Object(0,'', Pose(Point(),Quaternion()), Vector3())
#        )
#        self.arms.start_move_to_pose(arm_state, arm_index)


#        xMin = 1000
#        xMax = 9000
#        yMin = 0
#        yMax = 9000
#        xArray = [0 for x in xrange (81)]
#        yArray = [0 for x in xrange (81)]
#        matrix = [[[0 for x in xrange(2)] for y in xrange(9)] for z in xrange(9)] 
        
#        getCoords(xMin, xMax, yMin, yMax, matrix, xArray, yArray)
#        colorArray = ["Black" for x in xrange (81)]
        
    #    print "Matrix: ", matrix
    #    print "xArray: ", xArray
    #    print "yArray: ", yArray
    #    print "ColorArray: ", colorArray
        

#        for i in range (0, 81):
#            getBlock(i, colorArray[i])
#            placeBlock(i, xArray[i], yArray[i])

        return [RobotSpeech.OBJECT_NOT_DETECTED, GazeGoal.SHAKE]


    def moveToPos(self, x, y, z):
        print "Moving to", x, y, z

    def sayColor(self, color):
        raw_input ("Please give block of " + color + " color, and press enter afterwards")

    def grabBlock(self, i):
        print "Grabbing block ", i, " now"

    def releaseBlock(self, i):
        print "Placing block ", i, " now"  
        
    def getBlock(self, i, color):
        # sideways position
        moveToPos(0, 0, 0)
        sayColor(color)
        grabBlock(i)
        
    def placeBlock(self, i, x, y):
        # Move above the required grid
        moveToPos(x, y, 100)
        # Move closer to the table
        moveToPos(x, y, 10)
        # Release gripper
        releaseBlock(i)
        # Move up again
        moveToPos(x, y, 100)    

    def getCoords(self, xMin, xMax, yMin, yMax, matrix, xArray, yArray):
        xRange = xMax-xMin
        yRange = yMax- yMin
        rangeAvailable = min (xRange, yRange)
        blockSize = rangeAvailable/8
        xStart = xMin
        yStart = yMin
        
        for i in range (0, 9):
            for j in range (0, 9):
                matrix[i][j][0] = xStart + j*blockSize
                matrix[i][j][1] = yStart + i*blockSize
                
                xArray[9*i+j] = xStart + j*blockSize
                yArray[9*i+j] = yStart + i*blockSize

    def save_experiment_state(self):
        '''Saves session state'''
        self.session.save_current_action()

    @staticmethod
    def empty_response(responses):
        '''Default response to speech commands'''
        return responses
