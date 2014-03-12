'''Main interaction loop'''

import roslib
roslib.load_manifest('pr2_pbd_interaction')

# Generic libraries
import rospy
import os
import shutil
import time
import yaml
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


class Interaction:
    '''Finite state machine for the human interaction'''

    _is_programming = True
    _is_recording_motion = False
    _arm_trajectory = None
    _trajectory_start_time = None

    def __init__(self):
        # We need the participant ID before we construct the Session as it's
        # used in world.get_frame_list, so we're migrating part of it here.
        is_reload = rospy.get_param('/pr2_pbd_interaction/isReload')
        is_debug = False # Manually set this
        if (is_debug):
            exp_number = rospy.get_param(
                '/pr2_pbd_interaction/experimentNumber')
            rospy.set_param('experiment_number', exp_number)
            data_dir = Interaction._get_data_dir(exp_number)
            rospy.set_param('data_directory', data_dir)
            if (not os.path.exists(data_dir)):
                os.makedirs(data_dir)
        else:
            is_reload = True
            self._get_participant_id()

        # Old init starts here
        self.arms = Arms()
        self.world = World()
        # NOTE(max): Can't get the current action number from the session
        # becauase we're creating it. More importantly, the world needs to
        # access the fully-initialized session before it can mock objects (which
        # happens in get_frame_list()) and it's can't yet, so just use action 0
        # to return an empty object list back.
        self.session = Session(object_list=self.world.get_frame_list(1),
            is_reload=is_reload)
        self._viz_publisher = rospy.Publisher('visualization_marker_array',
            MarkerArray)

        rospy.Subscriber('recognized_command', Command, self.speech_command_cb)
        rospy.Subscriber('gui_command', GuiCommand, self.gui_command_cb)

        self._undo_function = None
        # NOTE(max): For counting frequency of pings... want to do not every 0.1
        # seconds.
        self._update_counter = 0

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

        rospy.loginfo('Interaction initialized.')

    @staticmethod
    def _get_participant_id():
        '''Gets the experiment number from the command line'''
        # NOTE(max): We always either reload data that's already been generated,
        # ore we generate it ourselves and then 'reload' it.
        exp_number = None
        data_dir = None
        while (exp_number == None):
            try:
                exp_number = int(raw_input(
                                    'Please enter participant ID:'))
            except ValueError:
                rospy.logerr("Participant ID needs to be a number")
                exp_number = None
                continue

            data_dir = Interaction._get_data_dir(exp_number)
            generate_files = False
            if (os.path.exists(data_dir)):
                rospy.logwarn('A directory for this participant ' +
                  'ID already exists: ' + data_dir)
                overwrite = raw_input('Do you want to overwrite? ' +
                  'Type r to reload the last state ' +
                  'of the experiment. [y/n/r]')
                if (overwrite == 'y'):
                    # Generate the files and overwrite existing data
                    generate_files = True
                elif (overwrite == 'n'):
                    # Ask for number again
                    exp_number = None
                    continue
                elif (overwrite == 'r'):
                    # Don't generate files; reload them
                    pass
                else:
                    # Ask for number again
                    rospy.logerr('Invalid response, try again.')
                    exp_number = None
                    continue
            else:
                # If the directory doesn't exist, make it and generate files.
                os.makedirs(data_dir)
                generate_files = True
            if generate_files:
                # Copy particular seed's actions n_tests times each
                n_tests = rospy.get_param('/pr2_pbd_interaction/nTests')
                seed_dir = Interaction._get_seed_dir(exp_number)
                seed_actions = sorted(os.listdir(seed_dir))
                n_tasks = int(rospy.get_param('/pr2_pbd_interaction/nTasks'))
                if len(seed_actions) != n_tasks:                    
                    rospy.logwarn("Have specified " + str(n_tasks) + " but " +
                        "there are " + str(len(seed_actions)) + " seeds...")
                cur_idx = 1
                # Note that seed_actions should have the same length as the
                # rospy param
                for seed_action in seed_actions:
                    for i in range(n_tests):
                        shutil.copy(seed_dir + seed_action,
                            data_dir + 'Action' + str(cur_idx) + '.bag')
                        cur_idx += 1
                # write experiment state
                exp_state = dict()
                exp_state['nProgrammedActions'] = cur_idx - 1
                exp_state['currentProgrammedActionIndex'] = 1
                state_file = open(data_dir + 'experimentState.yaml', 'w')
                state_file.write(yaml.dump(exp_state))
                state_file.close()
        # Save the parameters for global access
        rospy.set_param('data_directory', data_dir)
        rospy.set_param('experiment_number', exp_number)
    
    @staticmethod
    def _get_data_dir(exp_number):
        '''Returns the directory where action information is saved'''
        return (rospy.get_param('/pr2_pbd_interaction/dataRoot') +
                    '/data/experiment' + str(exp_number) + '/')

    @staticmethod
    def _get_seed_number(exp_number):
        ''' Maps experiment number to seed nubmer'''
        options = os.listdir(Interaction._get_root_seed_dir())
        n_seeds = len(options)
        return (exp_number % n_seeds) + 1

    @staticmethod
    def _get_root_seed_dir():
        '''Gets the directory that contains seed directories'''
        return rospy.get_param('/pr2_pbd_interaction/dataRoot') + '/data/seed/'

    @staticmethod
    def _get_seed_dir(exp_number):
        '''Gets the seed directory (containing the seed files) for the
        given experiment number'''
        return Interaction._get_root_seed_dir() + \
            str(Interaction._get_seed_number(exp_number)) + '/'


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
            if self.session.next_action(self.world.get_frame_list(
                self.session.current_action_index + 1)):
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
            if self.session.previous_action(self.world.get_frame_list(
                self.session.current_action_index - 1)):
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
                    self.world.get_frame_list(
                        self.session.current_action_index))

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
                self.world.get_frame_list(
                    self.session.current_action_index))
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
        ref_names = self.world.get_frame_list(self.session.current_action_index)
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
                    self.world.get_frame_list(
                        self.session.current_action_index))
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
                # print "\n\nDEBUG"
                # print "arm index: " + str(arm_index)
                # print "abs ee poses for it: " + str(abs_ee_poses[arm_index])
                # print"\n\n"
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
                            self.world.get_frame_list(
                                self.session.current_action_index))
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
                        self.world.get_frame_list(action_no))
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
                    # Let's generate all the rest of the experiment data!
                    for loop in range(20):
                        # Possibly increment gen. directory (if all objects
                        # generated)
                        World.maybe_inc_gen_no()

                        # Now looping to generate everything
                        for action_no in range(1, 16):
                            obj_filename = World.get_objfilename_for_action(
                                action_no)
                            if os.path.exists(obj_filename):
                                # We've already mocked the objects, so we just load it
                                # up and let the user keep working on them.
                                self.session.switch_to_action(action_no,
                                    self.world.get_frame_list(action_no))
                            else:
                                # We need to mock the objects until we get the desired
                                # number of fixable poses.
                                desired_n_unreachable = World._get_desired_n_unreachable(
                                    action_no)                        
                                rospy.loginfo("Sampling until " + str(
                                    desired_n_unreachable) + " unreachables.")
                                while True:
                                    self.session.switch_to_action(action_no,
                                        self.world.get_frame_list(action_no))
                                    n_unreachable = self.session\
                                        .get_cur_n_unreachable_markers()
                                    if n_unreachable == desired_n_unreachable:
                                        break
                                    else:
                                        rospy.loginfo("Sampled with " + str(
                                            n_unreachable) + " unreachable, but " +
                                            "wanted " + str(desired_n_unreachable))
                                # Save the objects so we don't have to sample again.
                                self.world.write_cur_objs_to_file(action_no)
                            response = Response(Interaction.empty_response,
                                [RobotSpeech.SWITCH_SKILL + str(action_no),
                                 GazeGoal.NOD])
                            response.respond()
                        rospy.loginfo('Finished loop ' + str(loop) + ' / 20.')
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
                    self.world.get_frame_list(
                        self.session.current_action_index))

        # Only ping state every 1 second or so.
        if self._update_counter % 10 == 0:
            #rospy.loginfo('pinging...')
            #self.session.ping_state()
            #rospy.loginfo('...done pinging')
            pass

        # Save all actions every 10 seconds or so.
        if self._update_counter == 0:
            self.session.save_session_state(True) # Save all actions.

        # Loop every 10 seconds
        self._update_counter = 0 if self._update_counter >= 100 else \
            self._update_counter + 1

        # Note that timings above depend on this... should probably make
        # a constant.
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
        if (self.world.update_object_pose()):
            if (self.session.n_actions() > 0):
                self.session.get_current_action().update_objects(
                    self.world.get_frame_list(
                        self.session.current_action_index))
            return [RobotSpeech.START_STATE_RECORDED, GazeGoal.NOD]
        else:
            return [RobotSpeech.OBJECT_NOT_DETECTED, GazeGoal.SHAKE]

    def save_experiment_state(self):
        '''Saves session state'''
        self.session.save_current_action()

    @staticmethod
    def empty_response(responses):
        '''Default response to speech commands'''
        return responses
