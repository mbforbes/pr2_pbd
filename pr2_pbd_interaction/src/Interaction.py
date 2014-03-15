'''Main interaction loop'''

import roslib
roslib.load_manifest('pr2_pbd_interaction')

# Generic libraries
import glob
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
        # Moving items from old init up here so we can use world in conditional
        # init code below.
        self.arms = Arms()
        self.world = World()

        # We need the participant ID before we construct the Session as it's
        # used in world.get_frame_list, so we're migrating part of it here.        
        mode = rospy.get_param('/pr2_pbd_interaction/mode')
        objects = []
        if mode == 'debug':
            is_reload = rospy.get_param('/pr2_pbd_interaction/isReload')
            exp_number = rospy.get_param(
                '/pr2_pbd_interaction/experimentNumber')
            data_dir = Interaction._get_data_dir(exp_number)
            if (not os.path.exists(data_dir)):
                os.makedirs(data_dir)
            rospy.set_param('data_directory', data_dir)
        elif mode == 'study':
            # We always reload becuase we are either (a) actually realoading, or
            # (b) copying in the seeds and then 'reloading' them for editing.
            is_reload = True
            # This function sets the rospy params experiment_number and
            # data_directory.
            self._get_participant_id()
        elif mode == 'analysis':
            # We'll be reloading some deafult / dummy action first.
            is_reload = True
            data_dir = rospy.get_param('/pr2_pbd_interaction/dataRoot') + \
                '/data/experimentAnalysis/'
            if (not os.path.exists(data_dir)):
                os.makedirs(data_dir)
            rospy.set_param('data_directory', data_dir)

            # Copy over default first test so that things work.
            default_dir = rospy.get_param('/pr2_pbd_interaction/dataRoot') + \
                'data/objects/test/gentest_1'
            rospy.set_param('da_obj_directory', default_dir)
            rospy.set_param('da_obj_filename', 'Action1.txt')
            objects = self.world.get_frame_list(1)

            # Copy over seed action to load so that things like getting the
            # experiment state in the session work. (Otherwise GUI can't start,
            # etc.)
            default_bag = Interaction._get_root_seed_dir() + '1/Action1.bag'
            shutil.copy(default_bag, data_dir + 'Action1.bag')

            # Make logfile
            lognum = 1
            while os.path.exists(data_dir + 'log_' + str(lognum) + '.txt'):
                lognum += 1
            self.logfile = data_dir + 'log_' + str(lognum) + '.txt'

            # Generate the experimentState.yaml (dummy; just 1,1)
            exp_state = dict()
            exp_state['nProgrammedActions'] = 1
            exp_state['currentProgrammedActionIndex'] = 1
            state_file = open(data_dir + 'experimentState.yaml', 'w')
            state_file.write(yaml.dump(exp_state))
            state_file.close()
        else:
            # This may be overkill, but I'd rather this for now that dealing
            # with weird errors later.
            rospy.logfatal('Must specify program mode. Killing node.')
            exit(1)


        # Old init continues here...

        # NOTE(max): Can't get the current action number from the session
        # becauase we're creating it. More importantly, the world needs to
        # access the fully-initialized session before it can mock objects (which
        # happens in get_frame_list()) and it can't yet, so just use action 0
        # to return an empty object list back.
        self.session = Session(object_list=objects, is_reload=is_reload)

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

        # NOTE: running on GUI click instead
        #if mode == 'analysis':
        #    self._run_feasibility_analysis()

        rospy.loginfo('Interaction initialized.')

    def _run_feasibility_analysis(self):
        ''' Runs feasibility analysis on collected data to objects in 
        [data root]/data/objects/test/'''
        # settings
        tasks = [1,2,3]
        prefix = 'gentest_' # all test object directories start with this
        # Implict: only one seed (so there is just a single seed directory 1/)
        # Implict: test objects are numbered 1-15, 5 each for each task
        dirs_to_remove = [
            'experiment1', # my 'experiment' for testing things out
            'experiment2', # first (2) experiment didn't use generated data
            'experiment3', # second (3) experiment didn't use generated data
            'experimentAnaylsis' # where the analysis data is cached
        ]
        root_dir = rospy.get_param('/pr2_pbd_interaction/dataRoot') + '/'

        # Print format
        self.log = open(self.logfile, 'w')
        # New format
        self.log.write('task_no,no_unreachable_before_fixing,test_dir_no,' +
            'test_action_no,user_no,user_action_no,no_unreachable_result,' +
            'user_orig_no_unreachable\n')
        # Original format
        #self.log.write('test_no,user_no,scenario_no,n_unreachable\n')

        # Loop tasks
        for task in tasks:
            # TODO change this once restructured (and other crap warnings gone).
            rospy.loginfo('- Running task ' + str(task) + ' of ' +
                str(max(tasks)))

            # Loop through test dirs
            test_root_dir = rospy.get_param('/pr2_pbd_interaction/dataRoot') + \
                '/data/objects/test/'
            # Still makes the '10 before 1' sorting bug, but mreh...
            test_dirs = sorted([d + '/' for d in glob.glob(test_root_dir +
                prefix + '*')])
            for test_dir in test_dirs:
                # Maps 1 -> 1,2,3,4,5, 2 - > 6,7,8,9,10, etc.
                filenames = ['Action' + str(i) + '.txt' for i in
                    range((task - 1) * 5 + 1, task * 5 + 1)]
                # Loop test cases
                for testfile in filenames:
                    # Get user data
                    globpath = root_dir + 'data/experiment*'
                    user_dirs = sorted([d + '/' for d in glob.glob(globpath)])
                    # Debug...
                    #rospy.loginfo('Globbing ' + globpath + '; got: ' +
                    #    str(user_dirs) + '\n')

                    # Clean
                    user_dirs[:] = [u for u in user_dirs if u.split('/')[-2] \
                        not in dirs_to_remove]

                    # Test the seed (currently assuming just one seed directory
                    # 1/)
                    seed_bag = Interaction._get_root_seed_dir() + '1/Action' + \
                        str(task) + '.bag'
                    # Note: as a sanity check, for good test cases (e.g. those
                    # we auto-generate), THESE SHOULD NEVER RETURN 0 (for a
                    # seed).
                    self._do_feasability_test(task, test_dir, testfile,
                        seed_bag)

                    # Loop users
                    for user_dir in user_dirs:
                        bags = sorted(glob.glob(user_dir + '*.bag'))
                        if len(bags) != 15:
                            # Some directories don't have user data in them yet.
                            continue
                        valid_bag_endings = ['Action' + str(i) + '.bag' for i \
                            in range((task - 1) * 5 + 1, task * 5 + 1)]
                        valid_bags = []
                        for b in bags:
                            for vbe in valid_bag_endings:
                                if b.endswith(vbe):
                                    valid_bags += [b]
                                    break
                        # Loop user's fixes (scenarios)
                        for bag in sorted(valid_bags):
                            pass
                            self._do_feasability_test(task, test_dir,
                                testfile, bag)

        # Cleanup
        self.log.close()

    def _do_feasability_test(self, task, test_dir, testfile, bagfile):
        '''Copies bagfile (either fixed data from user or seed) into the data
        directory for this task, loads it, and then checks and logs how many
        unreachable markers there are.'''
        # Set the objects to be mocked as those in the testfile
        #raw_input('Testing:\n\ttest: ' + testfile + '\n\tbag: ' + bagfile +
        #    '\n\t' + 'press ENTER to continue...')
        rospy.set_param('da_obj_directory', test_dir)
        rospy.set_param('da_obj_filename', testfile)

        # We always copy into Action1.bag
        dest = rospy.get_param('data_directory') + 'Action1.bag'
        shutil.copy(bagfile, dest)

        # Provide dummy val (1) for action_index to get_frame_list because it's
        # just going to use the rospy setting above (the parameter
        # da_obj_filename).
        self.session.reload_session_state(self.world.get_frame_list(1))
        # I think I need to actually call switch_to_action in order to get any
        # kind of results?
        self.session.switch_to_action(1, self.world.get_frame_list(1))
        # Doing the main loop's update here (to avoid race conditions).
        self.update()
        # This computes the 'actual' test data.
        n_unreachable = self.session.get_cur_n_unreachable_markers()

        # extract info for logging (one line each, comma-separated)
        # format:
        #  - task
        #  - no. unreachable before fixing
        #  - test dir no.
        #  - test action no. (test_no)
        #  - user no.
        #  - user action no. (scenario no.)
        #  - no. unreachable result (n_unreachable)
        #  - user orig. no. unreachable (for user's action)

        # '.../gentest_5/' -> '5'
        test_dir_no = test_dir.split('/')[-2].split('_')[-1]
        # 'Action14.txt' -> '14'
        test_no = testfile.split('.')[0].split('n')[1]
        # Gets how many poses were unreachable for this test before fixing
        n_unreachable_before_fix = \
            Interaction._get_seed_unreachable_from_action_no(int(test_no))
        if bagfile.find('seed') > -1:
            # Seed
            # We aren't logging seeds, we're just doing assertions here to
            # ensure everything's in order.
            if n_unreachable_before_fix != n_unreachable:
                self.log.write('ERROR: SEED FIXABILITY MISMATCH!')

            # Originally we logged this...
            #self.log.write(test_no + ',seed,' + str(n_unreachable) + '\n')
        else:
            # User / scenario
            # '.../experiment12/Action11.bag' -> '12'
            user_no = bagfile.split('/')[-2].split('t')[1]
            if int(user_no) < 4:
                self.log.write('WARNING: REMOVE USERS BETTER! FOUND ' + user_no)
                return

            # '.../experiment12/Action11.bag' -> '11'
            scenario_no = bagfile.split('/')[-1].split('.')[0].split('n')[1]
            # Gets how many poses the user's fix started with
            n_unreachable_user_orig = \
                Interaction._get_seed_unreachable_from_action_no(int(
                    scenario_no))

            self.log.write(str(task) + ',' + str(n_unreachable_before_fix) +
                ',' + test_dir_no + ',' + test_no + ',' + user_no + ',' + 
                scenario_no + ',' + str(n_unreachable) + ',' +
                str(n_unreachable_user_orig) + '\n')

            # Original format
            #self.log.write(test_no + ',' + user_no + ',' + scenario_no + ',' +
            #    str(n_unreachable) + '\n')
        # debug: doing this for faster feedback
        self.log.flush()

    @staticmethod
    def _get_seed_unreachable_from_action_no(action_no):
        '''Each action (1-15) is generated by sampling until a seed cannot reach
        a pre-specified number of poses. This takes the resulting action number
        (a number between 1 and 15) and maps it backwards to the number of
        unreachable poses for the seed (a number between 1 and 5).'''
        # NOTE: keep this in sync with World._get_desired_n_unreachable(...) !
        return [0, 2, 1, 3, 1, 2, 4, 2, 3, 5, 1, 3, 1, 2, 5, 4][action_no]


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
                    action_no = command.param
                    mode = rospy.get_param('/pr2_pbd_interaction/mode')
                    if mode == 'analysis':
                        # Hack: in analysis mode, clicking on any action runs
                        # the full feasability analysis (all tasks, all tests,
                        # all user / seed data).
                        self._run_feasibility_analysis()
                    else:
                        # NOTE(max): I'm doing the fixing up here but not in the
                        # speech_command_cb hook or any of the others... this is
                        # what we got for having non-refactored code... I would
                        # refactor now but don't have time.
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

        # Save all actions every 10 seconds or so. OFF FOR TESTING
        #if self._update_counter == 0:
        #    self.session.save_session_state(True) # Save all actions.

        # Loop every 10 seconds OFF DONT NEED
        #self._update_counter = 0 if self._update_counter >= 100 else \
        #    self._update_counter + 1

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
