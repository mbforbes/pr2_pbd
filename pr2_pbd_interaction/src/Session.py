'''Everything related to an experiment session'''

from ProgrammedAction import ProgrammedAction
import rospy
import os
import shutil
import time
import yaml
from pr2_pbd_interaction.msg import ExperimentState
from pr2_pbd_interaction.srv import GetExperimentState
from pr2_pbd_interaction.srv import GetExperimentStateResponse


class Session:
    '''This class holds and maintains experimental data'''

    def __init__(self, object_list, is_debug=False):
        self._is_reload = rospy.get_param('/pr2_pbd_interaction/isReload')
        self._n_tests = rospy.get_param('/pr2_pbd_interaction/nTests')
        self._seed_number = None
        self._exp_number = None
        self._selected_step = 0
        self._object_list = object_list

        if (is_debug):
            self._exp_number = rospy.get_param(
                                '/pr2_pbd_interaction/experimentNumber')
            self._data_dir = self._get_data_dir(self._exp_number)
            if (not os.path.exists(self._data_dir)):
                os.makedirs(self._data_dir)
        else:
            self._get_participant_id()
        rospy.set_param('data_directory', self._data_dir)

        self.actions = dict()
        self.current_action_index = 0

        if (self._is_reload):
            self._load_session_state(object_list)
            rospy.loginfo("Session state loaded.")

        n_actions = dict()
        for k in self.actions.keys():
            n_actions[str(k)] = self.actions[k].n_frames()

        self._state_publisher = rospy.Publisher('experiment_state',
                                                ExperimentState)
        rospy.Service('get_experiment_state', GetExperimentState,
                      self.get_experiment_state_cb)

        self._update_experiment_state()

    def get_cur_n_unreachable_markers(self):
        '''Function to allow external querying of number of unreachable
        markers.'''
        return (self.actions[self.current_action_index]
            .get_n_unreachable_markers())

    def ping_state(self):
        '''Used to push experiment service state more often (but not absurdly
            often) to gui.'''
        self._update_experiment_state()

    def _selected_step_cb(self, selected_step):
        '''Updates the selected step when interactive
        markers are clicked on'''
        self._selected_step = selected_step
        self._update_experiment_state()

    def get_experiment_state_cb(self, dummy):
        ''' Response to the experiment state service call'''
        return GetExperimentStateResponse(self._get_experiment_state())

    def _update_experiment_state(self):
        ''' Publishes a message with the latest state'''
        state = self._get_experiment_state()
        self._state_publisher.publish(state)

    def _get_experiment_state(self):
        ''' Creates a message with the latest state'''
        return ExperimentState(self.n_actions(),
                    self.current_action_index,
                    self.n_frames(),
                    self._selected_step,
                    self._get_gripper_states(0),
                    self._get_gripper_states(1),
                    self._get_ref_frames(0),
                    self._get_ref_frames(1),
                    self._object_list,
                    self.actions[self.current_action_index].
                        get_n_unreachable_markers())

    def _get_ref_frames(self, arm_index):
        ''' Returns the reference frames for the steps of the
        current action in array format'''
        ref_frames = []
        for i in range(self.n_frames()):
            action = self.actions[self.current_action_index]
            ref_frame = action.get_step_ref_frame(arm_index, i)
            ref_frames.append(ref_frame)
        return ref_frames

    def _get_gripper_states(self, arm_index):
        ''' Returns the gripper states for current action
        in array format'''
        gripper_states = []
        for i in range(self.n_frames()):
            action = self.actions[self.current_action_index]
            gripper_state = action.get_step_gripper_state(arm_index, i)
            gripper_states.append(gripper_state)
        return gripper_states

    def select_action_step(self, step_id):
        ''' Makes the interactive marker for the indicated action
        step selected, by showing the 6D controls'''
        self.actions[self.current_action_index].select_step(step_id)
        self._selected_step = step_id

    def _get_participant_id(self):
        '''Gets the experiment number from the command line'''
        # NOTE(max): We always either reload data that's already been generated,
        # ore we generate it ourselves and then 'reload' it.
        self._is_reload = True
        while (self._exp_number == None):
            try:
                self._exp_number = int(raw_input(
                                    'Please enter participant ID:'))
            except ValueError:
                rospy.logerr("Participant ID needs to be a number")
                self._exp_number = None
                continue

            self._data_dir = Session._get_data_dir(self._exp_number)
            generate_files = False
            if (os.path.exists(self._data_dir)):
                rospy.logwarn('A directory for this participant ' +
                  'ID already exists: ' + self._data_dir)
                overwrite = raw_input('Do you want to overwrite? ' +
                  'Type r to reload the last state ' +
                  'of the experiment. [y/n/r]')
                if (overwrite == 'y'):
                    # Generate the files and overwrite existing data
                    generate_files = True
                elif (overwrite == 'n'):
                    # Ask for number again
                    self._exp_number = None
                    continue
                elif (overwrite == 'r'):
                    # Don't generate files; reload them
                    pass
                else:
                    # Ask for number again
                    rospy.logerr('Invalid response, try again.')
                    self._exp_number = None
                    continue
            else:
                # If the directory doesn't exist, make it and generate files.
                os.makedirs(self._data_dir)
                generate_files = True
            if generate_files:
                # Copy particular seed's actions _n_tests times each
                self._seed_dir = Session._get_seed_dir(self._exp_number)
                seed_actions = sorted(os.listdir(self._seed_dir))
                n_tasks = int(rospy.get_param('/pr2_pbd_interaction/nTasks'))
                if len(seed_actions) != n_tasks:                    
                    rospy.logwarn("Have specified " + str(n_tasks) + " but " +
                        "there are " + str(len(seed_actions)) + " seeds...")
                cur_idx = 1
                # Note that seed_actions should have the same length as the
                # rospy param
                for seed_action in seed_actions:
                    for i in range(self._n_tests):
                        shutil.copy(self._seed_dir + seed_action,
                            self._data_dir + 'Action' + str(cur_idx) + '.bag')
                        cur_idx += 1
                # write experiment state
                exp_state = dict()
                exp_state['nProgrammedActions'] = cur_idx - 1
                exp_state['currentProgrammedActionIndex'] = 1
                # TODO(max): Need to save action completion (fix) state here?
                state_file = open(self._data_dir + 'experimentState.yaml', 'w')
                state_file.write(yaml.dump(exp_state))
                state_file.close()

    @staticmethod
    def _get_data_dir(exp_number):
        '''Returns the directory where action information is saved'''
        return (rospy.get_param('/pr2_pbd_interaction/dataRoot') +
                    '/data/experiment' + str(exp_number) + '/')

    @staticmethod
    def _get_seed_number(exp_number):
        ''' Maps experiment number to seed nubmer'''
        options = os.listdir(Session._get_root_seed_dir())
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
        return Session._get_root_seed_dir() + \
            str(Session._get_seed_number(exp_number)) + '/'

    def save_session_state(self, is_save_actions=True):
        '''Saves the session state onto hard drive'''
        savemsg = 'Saving session state'
        if is_save_actions:
            savemsg += ' and all ' + str(len(self.actions)) + ' actions'
        rospy.loginfo(savemsg)
        exp_state = dict()
        exp_state['nProgrammedActions'] = self.n_actions()
        exp_state['currentProgrammedActionIndex'] = self.current_action_index
        # TODO(max): Need to save action completion (fix) state here?
        state_file = open(self._data_dir + 'experimentState.yaml', 'w')
        state_file.write(yaml.dump(exp_state))
        state_file.close()

        if (is_save_actions):
            for num, action in self.actions.iteritems():
                action.save(self._data_dir)
        rospy.loginfo('...done saving')

    def _log_action_switch(self, new_action_idx):
        '''Dumps user action switch in a log file.'''
        action_log = open(self._data_dir + 'actionLog.txt', 'a')
        action_log.write(time.asctime() + ',' + str(new_action_idx) + '\n')
        action_log.close()

    def _load_session_state(self, object_list):
        '''Loads the experiment state from the hard drive'''
        state_file = open(self._data_dir + 'experimentState.yaml', 'r')
        exp_state = yaml.load(state_file)
        n_actions = exp_state['nProgrammedActions']
        for i in range(n_actions):
            self.actions.update({(i + 1): ProgrammedAction(i + 1,
                                            self._selected_step_cb)})
            self.actions[(i + 1)].load(self._data_dir)
        self.current_action_index = exp_state['currentProgrammedActionIndex']
        # Log the initial action we started on as a 'switch'
        self._log_action_switch(self.current_action_index)
        self.actions[self.current_action_index].initialize_viz(object_list)
        state_file.close()

    def new_action(self):
        '''Creates new action'''
        if (self.n_actions() > 0):
            self.get_current_action().reset_viz()
        self.current_action_index = self.n_actions() + 1
        self.actions.update({self.current_action_index:
                             ProgrammedAction(self.current_action_index,
                                              self._selected_step_cb)})
        self._update_experiment_state()

    def n_actions(self):
        '''Returns the number of actions programmed so far'''
        return len(self.actions)

    def get_current_action(self):
        '''Returns the current action'''
        return self.actions[self.current_action_index]

#     def get_current_action_name(self):
#         return self.actions[self.current_action_index].get_name()

    def clear_current_action(self):
        '''Removes all steps in the current action'''
        if (self.n_actions() > 0):
            self.actions[self.current_action_index].clear()
        else:
            rospy.logwarn('No skills created yet.')
        self._update_experiment_state()

    def undo_clear(self):
        '''Undo the effect of clear'''
        if (self.n_actions() > 0):
            self.actions[self.current_action_index].undoClear()
        else:
            rospy.logwarn('No skills created yet.')
        self._update_experiment_state()

    def save_current_action(self):
        '''Save current action onto hard drive'''
        if (self.n_actions() > 0):
            self.actions[self.current_action_index].save(self._data_dir)
            self.save_session_state(is_save_actions=False)
        else:
            rospy.logwarn('No skills created yet.')

    def add_step_to_action(self, step, object_list):
        '''Add a new step to the current action'''
        if (self.n_actions() > 0):
            self.actions[self.current_action_index].add_action_step(step,
                                                                object_list)
        else:
            rospy.logwarn('No skills created yet.')
        self._object_list = object_list
        self._update_experiment_state()

    def delete_last_step(self):
        '''Removes the last step of the action'''
        if (self.n_actions() > 0):
            self.actions[self.current_action_index].delete_last_step()
        else:
            rospy.logwarn('No skills created yet.')
        self._update_experiment_state()

    def resume_deleted_step(self):
        '''Resumes the deleted step'''
        if (self.n_actions() > 0):
            self.actions[self.current_action_index].resume_deleted_step()
        else:
            rospy.logwarn('No skills created yet.')
        self._update_experiment_state()

    def switch_to_action(self, action_number, object_list):
        '''Switches to indicated action'''
        if (self.n_actions() > 0):
            if (action_number <= self.n_actions() and action_number > 0):
                self.get_current_action().reset_viz()
                self.current_action_index = action_number
                self._log_action_switch(self.current_action_index)
                self.get_current_action().initialize_viz(object_list)
                success = True
            else:
                rospy.logwarn('Cannot switch to action '
                              + str(action_number))
                success = False
        else:
            rospy.logwarn('No skills created yet.')
            success = False
        self._object_list = object_list
        self._update_experiment_state()
        return success

    def next_action(self, object_list):
        '''Switches to next action'''
        # TODO(max): This should be refactored with switch_to_action(); don't
        # want to duplicate logic.
        if (self.n_actions() > 0):
            if (self.current_action_index < self.n_actions()):
                self.get_current_action().reset_viz()
                self.current_action_index += 1
                self._log_action_switch(self.current_action_index)
                self.get_current_action().initialize_viz(object_list)
                success = True
            else:
                success = False
        else:
            rospy.logwarn('No skills created yet.')
            success = False
        self._object_list = object_list
        self._update_experiment_state()
        return success

    def previous_action(self, object_list):
        '''Switches to previous action'''
        if (self.n_actions() > 0):
            if (self.current_action_index > 1):
                self.get_current_action().reset_viz()
                self.current_action_index -= 1
                self.get_current_action().initialize_viz(object_list)
                success = True
            else:
                success = False
        else:
            rospy.logwarn('No skills created yet.')
            success = False
        self._object_list = object_list
        self._update_experiment_state()
        return success

    def n_frames(self):
        '''Returns the number of frames'''
        if (self.n_actions() > 0):
            return self.actions[self.current_action_index].n_frames()
        else:
            rospy.logwarn('No skills created yet.')
            return 0
