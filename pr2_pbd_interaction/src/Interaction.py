'''Main interaction loop'''

# ROS
import roslib
roslib.load_manifest('pr2_pbd_interaction')
import rospy
from visualization_msgs.msg import MarkerArray

# Generic libraries
import glob
import os
import shutil
import threading
import time
import yaml

# 3rd party
import numpy as np

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
from pr2_pbd_interaction.msg import ScoreResult, ScoreResultList
from pr2_pbd_speech_recognition.msg import Command
from pr2_social_gaze.msg import GazeGoal


class Interaction:
    '''Finite state machine for the human interaction'''

    _is_programming = True
    _is_recording_motion = False
    _arm_trajectory = None
    _trajectory_start_time = None

    def __init__(self):
        # Settings/code for success testing
        # ======================================================================
        # Basic settings
        is_reload = True
        n_tasks = len(glob.glob(rospy.get_param(
            '/pr2_pbd_interaction/dataRoot') + '/data/experimentTesting/task*'))
        self.top_n = 5 # How many results to return from the score function.

        # Copy default bag files (seed)
        Interaction._copy_seeds(n_tasks)

        # Set up for current task
        self.task_no = 1
        self.score_func = -1 # start invalid; should click to load first
        rospy.set_param('data_directory', Interaction._get_data_dir(
            self.task_no))

        # Load feasability results
        logfile = rospy.get_param('/pr2_pbd_interaction/dataRoot') + '/data/' +\
            'experimentAnalysis/log3_archives/log_3.txt'
        self.fdata = self.load_feasibility_data(logfile)

        self._score_publisher = rospy.Publisher('score_result_list',
            ScoreResultList)

        # For reloading actions, need a lock
        self._update_lock = threading.Lock()

        # Disabling this stuff for now because we're just going to focus on
        # success testing here.
        # ======================================================================
        #is_debug = False # Manually set this
        #if (is_debug):
        #    data_dir = Interaction._get_data_dir(exp_number)
        #    rospy.set_param('data_directory', data_dir)
        #    if (not os.path.exists(data_dir)):
        #        os.makedirs(data_dir)
        #else:
        #    is_reload = True
        #    self._get_participant_id()


        # Old init starts here
        # ======================================================================
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

    # STATIC METHDOS
    # ==========================================================================

    @staticmethod
    def _copy_seeds(n_tasks):
        '''Copies seeds into the task directories as default bag files
        (for experiment testing (success testing), as well as writes experiment
        state yaml file.'''
        seed_dir = Interaction._get_seed_dir()
        for task_no in range(1, n_tasks + 1):
            task_dir = Interaction._get_data_dir(task_no)
            n_tests = len(glob.glob(task_dir + 'objects/Action*.txt'))
            for test_no in range(1, n_tests + 1):
                # copy seed for every test
                shutil.copy(seed_dir + 'Action' + str(task_no) + '.bag',
                    task_dir + 'Action' + str(test_no) + '.bag')
            # write exp state
            exp_state = dict()
            exp_state['nProgrammedActions'] = n_tests
            exp_state['currentProgrammedActionIndex'] = 1
            state_file = open(task_dir + 'experimentState.yaml', 'w')
            state_file.write(yaml.dump(exp_state))
            state_file.close()

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
                # NOTE parameter missing; not supporting this method in this
                # branch                
                n_tests = rospy.get_param('/pr2_pbd_interaction/nTests')
                seed_dir = Interaction._get_seed_dir()
                seed_actions = sorted(os.listdir(seed_dir))
                # NOTE parameter missing; not supporting this method in this
                # branch
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
    
    @staticmethod
    def _get_data_dir(task_no):
        '''Returns the directory where action information is saved'''
        return (rospy.get_param('/pr2_pbd_interaction/dataRoot') +
                    '/data/experimentTesting/task' + str(task_no) + '/')

    @staticmethod
    def _get_seed_dir():
        '''Gets the seed directory (containing the seed files) for the
        given experiment number'''
        return rospy.get_param('/pr2_pbd_interaction/dataRoot') +'/data/seed/1/'

    @staticmethod
    def empty_response(responses):
        '''Default response to speech commands'''
        return responses

    # MEMBER METHDOS
    # ==========================================================================

    def load_feasibility_data(self, logfile):
        '''Returns the feasibility data as a numpy array. Format:
        - [0] task
        - [1] no. unreachable before fixing
        - [2] test dir no.
        - [3] test action no. (test_no)
        - [4] user no.
        - [5] user action no. (scenario no.)
        - [6] no. unreachable result (n_unreachable)
        - [7] user orig. no. unreachable (for user's action)
        - [8] user's confidence in their fix for their action
        '''
        return np.genfromtxt(logfile, delimiter=',', dtype='int8',
            skip_header=1)

    def score_confidence(self, testdir, testact):
        '''Score function that selects from feasibile results and sorts by
        users' assigned confidence rating.

        Returns (userdir, useract), ScoreResultList
        '''
        col_task      = 0 # task number
        col_nun_start = 1 # number unreachable before fixing (this test action)
        col_testdir   = 2 # test directory
        col_testact   = 3 # test action (i.e. the "test")
        col_userdir   = 4 # user directory / user number
        col_useract   = 5 # user action (which they were fixing)
        col_nun_res   = 6 # the resulting n. unreachable from user fix -> test
        col_nun_user  = 7 # original n. unreachable that user's act. started w/
        col_userconf  = 8 # user's confidence in their fix for their action   

        # Narrow down to what we want.
        task_data = self.fdata[np.where(
            self.fdata[:, col_task] == self.task_no)]
        testdir_data = task_data[np.where(
            task_data[:, col_testdir] == testdir)]
        testact_data = testdir_data[np.where(
            testdir_data[:, col_testact] == testact)]

        # Get only those that are feasible.
        feasibile_data = testact_data[np.where(
            testact_data[:, col_nun_res] == 0)]

        # Sort by user confidence.
        # Thanks to http://stackoverflow.com/questions/2828059/sorting-arrays-
        #     in-numpy-by-column
        # http://stackoverflow.com/questions/6771428/most-efficient-way-to-
        #     reverse-a-numpy-array
        sorted_data = feasibile_data[feasibile_data[:,col_userconf]\
            .argsort()][::-1] # get reverse view with [::-1] to 'reverse sort'
        topn = sorted_data[:self.top_n,[col_userdir, col_useract, col_userconf]]

        # Create response.
        results = []
        for res in list(topn):
            userdir, useract, userconf = list(res)
            results += [ScoreResult(userdir, useract, userconf)]
        scoreResultList = ScoreResultList(results)

        # Extract top for immediate switching-to.
        userdir, useract = None, None
        if len(topn) > 0:
            userdir, useract, userscore = list(topn[0])

        return (userdir, useract), scoreResultList

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


    def get_cur_test_info(self):
        '''This is where all the hardcoded values go. Given that it knows

        - current task (self.task_no)
        - current action (self.session.current_action_index)

        returns tuple of the source

        - test directory
        - test action

        that this came from (where the objects were copied from).'''
        curtask = self.task_no - 1 # 1-based -> 0-based
        curact = self.session.current_action_index - 1 # 1-based -> 0-based

        dir_mapping = [
            [1, 1, 2, 2, 3, 1, 1, 2, 1, 2],
            [1, 2, 1, 2, 1, 2, 1, 2, 1, 2],
            [1, 2, 1, 2, 1, 2, 1, 2, 1, 2],
        ]

        action_mapping = [
            [2, 4, 2, 4, 2, 1, 5, 1, 3, 3],
            [5, 5, 2, 2, 3, 3, 1, 1, 4, 4],
            [2, 2, 3, 3, 1, 1, 5, 5, 4, 4]
        ]

        testdir = dir_mapping[curtask][curact]
        testact = action_mapping[curtask][curact]
        return testdir, testact

    def score(self):
        '''Call relevant score function, switch to top, and publish result.'''
        if (self.score_func == 0):
            testdir, testact = self.get_cur_test_info()
            top, scoreResultList = self.score_confidence(testdir, testact)
            userdir, useract = top
            if userdir is not None and useract is not None:
                self.load_action(userdir, useract)
        else:
            # Above are the only implemented.
            rospy.logwarn('Requested score function (' + str(self.score_func) +
                ') not implemented.')
            return
        self._score_publisher.publish(scoreResultList)

    def load_action(self, userdir, useract):
        '''Loads a userdir/useract action as the current action.'''
        # Need to lock against the update loop because this runs in another
        # thread and if the update happens while we're swaping out the action,
        # it crashes (as it looks into the guts of the action).
        self._update_lock.acquire()
        rospy.loginfo('Swapping in ' + str(userdir) + ', ' + str(useract))

        # find src
        src_dir = rospy.get_param('/pr2_pbd_interaction/dataRoot') + '/data/' +\
            'experiment' + str(userdir) + '/'
        srcfilename = 'Action' + str(useract) + '.bag'
        src = src_dir + srcfilename

        # find dst
        task_dir = Interaction._get_data_dir(self.task_no)
        dstfilename = 'Action' + str(self.session.current_action_index) + '.bag'
        dst = task_dir + dstfilename

        # copy
        shutil.copy(src, dst)

        # update
        self.session.swap_action(self.world.get_frame_list(
            self.session.current_action_index))

        # make the GUI's unreachability counter reflect the changes
        self.session.ping_state()  

        # release
        self._update_lock.release()

    def gui_command_cb(self, command):
        '''Callback for when a GUI command is received'''
        if (not self.arms.is_executing()):
            if (self.session.n_actions() > 0):
                if (command.command == GuiCommand.SELECT_RESULT):
                    result = command.param
                    userdir = result / 100
                    useract = result % 100
                    self.load_action(userdir, useract)
                if (command.command == GuiCommand.SELECT_SCORE_FUNC):
                    self.score_func = command.param
                    self.score()
                if (command.command == GuiCommand.SWITCH_TO_ACTION):
                    # NOTE(max): I'm doing the fixing up here but not in the
                    # speech_command_cb hook or any of the others... this is
                    # what we got for having non-refactored code... I would
                    # refactor now but don't have time.
                    action_no = command.param
                    obj_filename = World.get_objfilename_for_action(action_no)
                    if os.path.exists(obj_filename):
                        self.session.switch_to_action(action_no,
                            self.world.get_frame_list(action_no))
                        self.score()
                    else:
                        rospy.logwarn('Test file did not exist...')
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
        # Doing this as it's async and want to avoid conflicts with action
        # swapping happening in the interaction thread.
        self._update_lock.acquire()
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
        self._update_lock.release()                
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

