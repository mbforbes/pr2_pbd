'''Tracks a specific set of robot state attributes for NLP processing.

Gets updates to robot experiment state and provides access to the data
in an interpretable format.
'''

__author__ = 'mbforbes'

########################################################################
# IMPORTS
########################################################################

# ROS imports come first
import roslib
roslib.load_manifest('pr2_pbd_speech_recognition')
import rospy

# Standard library imports.
import pprint
import time

# Local imports.
from pr2_pbd_msgs.msg import Command
from pr2_pbd_msgs.msg import Side
from pr2_pbd_msgs.msg import GripperState, GripperStateChange
from pr2_pbd_msgs.msg import ArmMode, ArmModeChange
from pr2_pbd_msgs.msg import ArmMove
from pr2_pbd_msgs.msg import ExecutionStatus
from pr2_pbd_msgs.srv import GetGripperStates, GetArmModes, GetExecutionStatus

########################################################################
# CONSTANTS
########################################################################

# Each object is composed of one or more properties.
OBJ_DICT = {
    # One property
    # -------------
    'execution': {
        'properties': ['execution']
    },
    'microphone': {
        'properties': ['microphone']
    },
    'motion': {
        'properties': ['motion']
    },
    'action': {
        'properties': ['action']
    },
    'pose': {
        'properties': ['pose']
    },

    # Multiple properties
    # -------------------
    # Misc
    'command': {
        'properties': ['last', 'command']
    },
    'object-pose': {
        'properties': ['pose', 'object']
    },

    # Hands
    'right-hand': {
        'properties': ['right', 'hand']
    },
    'left-hand': {
        'properties': ['left', 'hand']
    },

    # Arms
    'right-arm': {
        'properties': ['right', 'arm']
    },
    'left-arm': {
        'properties': ['left', 'arm']
    },

    # Actions
    'new-action': {
        'properties': ['new', 'action']
    },
    'next-action': {
        'properties': ['next', 'action']
    },
    'previous-action': {
        'properties': ['previous', 'action']
    },

    # Steps
    'all-step': {
        'properties': ['all', 'step']
    },
    'last-step': {
        'properties': ['last', 'step']
    },
}

# This maps commands in the system to NLP-defined 'objects'. Empty
# entries ('') do not map to an 'object' and thus are ignored.
CMD_OBJ_MAP = {
    Command.TEST_MICROPHONE: 'microphone',
    Command.RELAX_RIGHT_ARM: 'right-arm',
    Command.RELAX_LEFT_ARM: 'left-arm',
    Command.OPEN_RIGHT_HAND: 'right-hand',
    Command.OPEN_LEFT_HAND: 'left-hand',
    Command.CLOSE_RIGHT_HAND: 'right-hand',
    Command.CLOSE_LEFT_HAND: 'left-hand',
    Command.STOP_EXECUTION: 'execution',
    Command.UNDO: '', # TODO(max): Should we handle this?
    Command.DELETE_ALL_STEPS: 'all-step',
    Command.DELETE_LAST_STEP: 'last-step',
    Command.FREEZE_RIGHT_ARM: 'right-arm',
    Command.FREEZE_LEFT_ARM: 'left-arm',
    Command.RECORD_OBJECT_POSE: 'object-pose',
    Command.CREATE_NEW_ACTION: 'new-action',
    Command.EXECUTE_ACTION: 'execution',
    Command.NEXT_ACTION: 'next-action',
    Command.PREV_ACTION: 'previous-action',
    Command.SAVE_ACTION: 'action',
    Command.EDIT_ACTION: 'action',
    Command.SAVE_POSE: 'pose',
    Command.START_RECORDING_MOTION: 'motion',
    Command.STOP_RECORDING_MOTION: 'motion',
    Command.UNRECOGNIZED: ''
}

########################################################################
# CLASSES
########################################################################

class RobotState:
    '''The class that tracks and provides access to the robot state.'''

    def __init__(self):
        # Subscribe to relevant messages for state updates.
        rospy.Subscriber('recognized_command', Command, self.command_cb)
        rospy.Subscriber('gripper_state_change', GripperStateChange,
            self.gripper_state_change_cb)
        rospy.Subscriber('arm_mode_change', ArmModeChange,
            self.arm_mode_change_cb)
        rospy.Subscriber('arm_move', ArmMove, self.arm_move_cb)
        rospy.Subscriber('execution_status', ExecutionStatus,
            self.execution_status_cb)

        # NOTE(max): We're not subscribing to (gui_command, GuiCommand)
        # because we're assuming only speech interaction for this study.

        # TODO(max): Best place to document members? Here?

        # Copy the dictionary so we have our own mutable copy (not
        # mutating the global template). Set last-referred and
        # last-changed to default values.
        self.obj_dict = OBJ_DICT.copy()
        construct_time = time.time()
        for val in self.obj_dict.itervalues():
            # NOTE(max): Now not initializing times and just setting
            # when accessed.
            pass
            # val['last-referred'] = construct_time
            # val['last-changed'] = construct_time


        # Default gripper states / arm modes.
        self.gripper_states = [ArmMode.HOLD, ArmMode.HOLD]
        self.arm_modes = [GripperState.CLOSED, GripperState.CLOSED]
        self.execution_status = ExecutionStatus.NOT_EXECUTING

        # Initialize
        # - gripper states
        # - arm modes
        # - execution status
        # by calling an Arms service.
        rospy.wait_for_service('get_gripper_states')
        gripper_states_srv = rospy.ServiceProxy('get_gripper_states',
            GetGripperStates)
        try:
            grips = gripper_states_srv()
            self.gripper_states[Side.LEFT] = grips.left.state
            self.gripper_states[Side.RIGHT] = grips.right.state
        except rospy.ServiceException as exc:
            rospy.logwarn('get_gripper_states service could not process ' +
                'request: ' + str(exc))

        rospy.wait_for_service('get_arm_modes')
        arm_modes_srv = rospy.ServiceProxy('get_arm_modes', GetArmModes)
        try:
            modes = arm_modes_srv()
            self.arm_modes[Side.LEFT] = modes.left.mode
            self.arm_modes[Side.RIGHT] = modes.right.mode
        except rospy.ServiceException as exc:
            rospy.logwarn('get_arm_modes service could not process request: ' +
                str(exc))

        rospy.wait_for_service('get_execution_status')
        self.exec_status_srv = rospy.ServiceProxy('get_execution_status',
            GetExecutionStatus)
        try:
            self.execution_status = self.exec_status_srv().status.status
        except rospy.ServiceException as exc:
            rospy.logwarn('get_arm_modes service could not process request: ' +
                str(exc))


    ####################################################################
    # STATE-GETTING FUNCTIONS
    ####################################################################

    def get_state(self):
        '''Get a state object as specified by the NLP folks.

        /**
         * The last object in the system that changed.
         */
        private final Obj               lastChanged;


        /**
         * For each property in {@link Obj#getProperties()} the last object that
         * changed it status.
         */
        private final Map<String, Obj>  lastChangedByProperty;

        property : object

        e.x.

        left: left-arm || left-hand


        /**
         * The last object in the system that was referred by the user.
         */
        private final Obj               lastReferred;


        /**
         * For each property in {@link Obj#getProperties()} the last object referred
         * to by the user.
         */
        private final Map<String, Obj>  lastReferredByProperty;

        for all objects that include this property, which was the most
        recently referred to

        /**
         * For each {@link Obj} the current state.
         */
        private final Map<Obj, String>  stateByObj;

        - for each arm: frozen / relaxed
        - for each gripper: open / closed
        - exection: started / stopped


        Here's the list of all objects in the system and their properties:
        microphone --> [microphone]
        right-arm --> [right, arm]
        left-arm --> [left, arm]
        left-hand --> [left, hand]
        right-hand --> [right, hand]
        execution --> [execution]
        last-command --> [last, command]
        all-steps --> [all, step]
        last-step --> [last, step]
        object-pose --> [pose, object]
        new-action --> [new, action]
        next-action --> [next, action]
        previous-action --> [previous, action]
        action --> [action]
        pose --> [pose]
        motion --> [motion]

        The names of the two maps are:
        lastChangedByProperty
        lastReferredByProperty

        In addition, the state should still include:
        lastReferred
        lastChanged
        stateByObj


        When something's not implemented:
        =================================
        - 'NULL' (for string)
        - {} (for map)
        '''
        retobj = {}

        last_changed = self.get_last_changed()
        if last_changed is not '':
            retobj['lastChanged'] = last_changed

        last_referred = self.get_last_referred()
        if last_referred is not '':
            retobj['lastReferred'] = last_referred

        last_changed_by_property = self.get_last_changed_by_property()
        if len(last_changed_by_property) > 0:
            retobj['lastChangedByProperty'] = last_changed_by_property

        last_referred_by_property = self.get_last_referred_by_property()
        if len(last_referred_by_property) > 0:
            retobj['lastReferredByProperty'] = last_referred_by_property

        # State by obj should always be fine; always have robot state.
        state_by_obj = self.get_state_by_obj()
        retobj['stateByObj'] = state_by_obj

        return retobj

    def get_last_changed(self):
        '''The last object in the system that changed.

        returns string Obj
        '''
        return self.get_most_recent_selector('last-changed',
            list(self.obj_dict.iteritems()))

    def get_last_changed_by_property(self):
        '''For each property in {@link Obj#getProperties()} the last
        object that changed it status.

        returns {string: string} (Map<String, Obj>)
        '''
        return self.get_last_x_by_property('last-changed')

    def get_last_referred(self):
        '''The last object in the system that was referred by the user.

        returns string (obj)
        '''
        return self.get_most_recent_selector('last-referred',
            list(self.obj_dict.iteritems()))

    def get_last_referred_by_property(self):
        '''For each property in {@link Obj#getProperties()} the last
        object referred to by the user.

        returns {string: string} (Map<String, Obj>)
        '''
        return self.get_last_x_by_property('last-referred')

    def get_state_by_obj(self):
        '''For each {@link Obj} the current state.

        returns {string: string} (Map<Obj, String>)
        '''
        return {
            'right-hand': self.gripper_state_str(
                self.gripper_states[Side.RIGHT]),
            'right-arm': self.arm_mode_str(self.arm_modes[Side.RIGHT]),
            'left-hand': self.gripper_state_str(
                self.gripper_states[Side.LEFT]),
            'left-arm': self.arm_mode_str(self.arm_modes[Side.LEFT]),
            'execution': self.execution_status_str(self.execution_status)
        }

    ####################################################################
    # STATE-GETTING UTILITY FUNCTIONS
    ####################################################################

    def get_last_x_by_property(self, x):
        '''Generalized accessor.

        Args:
            x (string): 'last-referred' | 'last-changed'
        '''
        res = {}
        for prop in self.get_properties():
            tuples = [pair for pair in self.obj_dict.iteritems()
                    if prop in pair[1]['properties']]
            most_recent = self.get_most_recent_selector(x, tuples)
            if most_recent is not '':
                res[prop] = most_recent
        return res

    def get_most_recent_selector(self, selector, tuples):
        '''Returns the most recent object for a given selector (either
        last-referred or last-changed) and tuples (subset of flattened
        object dictionary).

        Args:
            selector (string): 'last-referred' | 'last-changed'

            tuples (list('string', dict of values)): subset of flattened
                                                     object dictionary

        Returns:
            string: name of specified most recent object
        '''
        # NOTE(max): This could be cleaner, but list comprehension
        # reduces are messy, and reduces over dictionaries are messy.
        # So, just going for a loop.
        last_time = 0 # OK baseline as default is construct time (> 0)
        best_key = ''
        for t in tuples:
            key, val = t[0], t[1]
            if val.has_key(selector) and val[selector] > last_time:
                best_key = key
                last_time = val[selector]
        return best_key

    def get_properties(self):
        '''Returns a list of all of the properties.

        Returns:
            list(str)
        '''
        all_props = set()
        for val in self.obj_dict.itervalues():
            all_props = all_props.union(set(val['properties']))
        return list(all_props)

    def gripper_state_str(self, gripper_state):
        '''Takes a gripper state integer and turns it into a string.

        Args:
            gripper_state (int): GripperState.OPEN | GripperState.CLOSED

        Returns:
            string: 'open' | 'closed'
        '''
        return ('open' if gripper_state == GripperState.OPEN else 'closed')

    def arm_mode_str(self, arm_mode):
        '''Takes an arm mode and turns it into a string.

        Args:
            arm_mode (int): ArmMode.RELEASE | ArmMode.HOLD

        Returns:
            string: 'frozen' | 'relaxed'
        '''
        return ('frozen' if arm_mode == ArmMode.HOLD else 'relaxed')

    def execution_status_str(self, execution_status):
        '''Takes an execution status and turns it into a string.

        Args:
            execution_status (int): ExecutionStatus.X

        Returns:
            string: 'started' | 'stopped'
        '''
        return ('started' if execution_status == ExecutionStatus.EXECUTING
            else 'stopped')

    ####################################################################
    # MESSAGE LISTENER / ROBOT-RESPONSIVE FUNCTIONS
    ####################################################################

    def command_cb(self, command):
        '''Callback for when we see a command is executed.

        Args:
            command (Command [Command.msg])
        '''
        nlp_obj = CMD_OBJ_MAP[command.command]
        if nlp_obj is not '':
            now = time.time()
            self.obj_dict[nlp_obj]['last-referred'] = now

    def gripper_state_change_cb(self, gripperStateChange):
        '''Callback for when the robot's gripper state changes (only
        when it actually happens; open -> open won't trigger this, for
        example)

        Args:
            gripperStateChange (GripperStateChange
                [GripperStateChange.msg])
        '''
        # Set this object as changed.
        now = time.time()
        side_raw = gripperStateChange.side.side
        state_raw = gripperStateChange.state.state
        if side_raw == Side.LEFT:
            self.obj_dict['left-hand']['last-changed'] = now
            print '[STATE] left-hand (gripper) most recent'
        else:
            self.obj_dict['right-hand']['last-changed'] = now
            print '[STATE] right-hand (gripper) most recent'

        # Change the saved state.
        self.gripper_states[side_raw] = state_raw
        print '[STATE] new gripper states:', self.gripper_states

    def arm_mode_change_cb(self, armModeChange):
        '''Callback for when the robot's arm mode changes (only when it
        actually happens; relaxed -> relaxed won't trigger this, for
        example)

        Args:
            armModeChange (ArmModeChange [ArmModeChange.msg])
        '''
        # Set this object as changed.
        now = time.time()
        side_raw = armModeChange.side.side
        mode_raw = armModeChange.mode.mode
        if side_raw == Side.LEFT:
            self.obj_dict['left-arm']['last-changed'] = now
            print '[STATE] left-arm (mode) most recent'
        else:
            self.obj_dict['right-arm']['last-changed'] = now
            print '[STATE] right-arm (mode) most recent'

        # Change saved state
        self.arm_modes[side_raw] = mode_raw
        print '[STATE] new arm modes:', self.arm_modes

    def arm_move_cb(self, armMove):
        '''Callback for when robot's arm moves (above some threshold).

        Args:
            armMove (ArmMove [ArmMove.msg])
        '''
        now = time.time()
        side_raw = armMove.side.side
        move_raw = armMove.delta
        if side_raw == Side.LEFT:
            self.obj_dict['left-arm']['last-changed'] = now
            print '[STATE] left-arm (moved) most recent'
        else:
            self.obj_dict['right-arm']['last-changed'] = now
            print '[STATE] right-arm (moved) most recent'

    def execution_status_cb(self, executionStatus):
        '''Callback for when execution status changes.

        Args:
            executionStatus (ExecutionStatus [ExecutionStatus.msg])
        '''
        now = time.time()
        status_raw = executionStatus.status
        self.execution_status = status_raw
        print '[STATE] new exec status:', self.execution_status
