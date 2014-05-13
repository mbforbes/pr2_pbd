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
import time

# Local imports.
from pr2_pbd_speech_recognition.msg import Command


########################################################################
# CONSTANTS
########################################################################

# Each object is composed of a type and optionally a property.
OBJ_DICT = {
    # No properties
    # -------------
    'execution': {
        'type': 'execution',
        'property': None
    },
    'microphone': {
        'type': 'microphone',
        'property': None
    },
    'motion': {
        'type': 'motion',
        'property': None
    },

    # One property
    # ------------
    'command': {
        'type': 'command',
        'property': 'last'
    },

    # Multiple properties
    # -------------------
    # Hands
    'right-hand': {
        'type': 'hand',
        'property': 'right'
    },
    'left-hand': {
        'type': 'hand',
        'property': 'left'
    },

    # Arms
    'right-arm': {
        'type': 'arm',
        'property': 'right'
    },
    'left-arm': {
        'type': 'arm',
        'property': 'left'
    },

    # Actions
    'new-action': {
        'type': 'action',
        'property': 'new'
    },
    'next-action': {
        'type': 'action',
        'property': 'next'
    },
    'previous-action': {
        'type': 'action',
        'property': 'previous'
    },
    'action': {
        'type': 'action',
        'property': None
    },

    # Poses
    'object-pose': {
        'type': 'pose',
        'property': 'object'
    },
    'pose': {
        'type': 'pose',
        'property': None
    },

    # Steps
    'all-step': {
        'type': 'step',
        'property': 'all'
    },
    'last-step': {
        'type': 'step',
        'property': 'last'
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

        # NOTE(max): We're not subscribing to (gui_command, GuiCommand)
        # because we're assuming only speech interaction for this study.

        # TODO(max): Best place to document members? Here?

        # Copy the dictionary so we have our own mutable copy (not
        # mutating the global template). Set last-referred and
        # last-changed to default values.
        self.obj_dict = OBJ_DICT.copy()
        construct_time = time.time()
        for val in self.obj_dict.itervalues():
            val['last-referred'] = construct_time
            val['last-changed'] = construct_time

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
         * For each {@link Obj#getLabel()}, the last object that changed its state.
         */
        private final Map<String, Obj>  lastChangedObjByType;

        arm: left-arm || right-arm


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
         * For each {@link Obj#getLabel()}, the last object referred to by the user.
         */
        private final Map<String, Obj>  lastReferredByType;

        for all objects that include this type, which was the most
        recently referred to


        /**
         * For each {@link Obj} the current state.
         */
        private final Map<Obj, String>  stateByObj;

        - for each arm: frozen / relaxed
        - for each gripper: open / closed
        - exection: started / stopped


        Enumeration of possibilities
        ============================

        Objects (16):
        -------------
        left-hand
        execution
        right-arm
        last-command
        new-action
        action
        microphone
        left-arm
        next-action
        motion
        pose
        object-pose
        right-hand
        last-step
        previous-action
        all-step

        Types (9):
        ----------
        execution
        motion
        pose
        action
        step
        microphone
        arm
        command
        hand

        Properties (8):
        ---------------
        next
        all
        new
        last
        previous
        left
        right
        object


        How types & properties combine to form objects:
        ===============================================

        No combination (3)
        ------------------
        - execution
        - microphone
        - motion

        Only one combination (1)
        ------------------------
        - command
            - last

        Multiple combinations (12)
        --------------------------
        - hand
            - left
            - right
        - arm
            - left
            - right
        - action
            - new
            - next
            - previous
            - ''
        - pose
            - ''
            - object
        - step
            - all
            - last


        When something's not implemented:
        =================================
        - 'NULL' (for string)
        - {} (for map)
        '''
        return {
            'lastChanged': self.get_last_changed(),
            'lastChangedByProperty': self.get_last_changed_by_property(),
            'lastChangedObjByType': self.get_last_changed_obj_by_type(),
            'lastReferred': self.get_last_referred(),
            'lastReferredByProperty': self.get_last_referred_by_property(),
            'lastReferredByType': self.get_last_referred_by_type(),
            'stateByObj': self.get_state_by_obj()
        }

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
        return self.get_last_x_by_y('last-changed', 'property')

    def get_last_changed_obj_by_type(self):
        '''For each {@link Obj#getLabel()}, the last object that changed
        its state.

        returns {string: string} (Map<String, Obj>)
        '''
        return self.get_last_x_by_y('last-changed', 'type')

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
        return self.get_last_x_by_y('last-referred', 'property')

    def get_last_referred_by_type(self):
        '''
        For each {@link Obj#getLabel()}, the last object referred to by
        the user.

        returns {string: string} (Map<String, Obj>)
        '''
        return self.get_last_x_by_y('last-referred', 'type')

    def get_state_by_obj(self):
        '''For each {@link Obj} the current state.

        returns {string: string} (Map<Obj, String>)
        '''
        # TODO(max): Implement.
        return {}

    ####################################################################
    # STATE-GETTING UTILITY FUNCTIONS
    ####################################################################

    def get_last_x_by_y(self, x, y):
        '''Generalized accessor.

        Args:
            x (string): 'last-referred' | 'last-changed'
            y (string): 'property' | 'type'
        '''
        res = {}
        for y_item in self.get_ys(y):
            tuples = [pair for pair in self.obj_dict.iteritems()
                if pair[1][y] == y_item]
            res[y_item] = self.get_most_recent_selector(x, tuples)
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
        for t in tuples:
            print t
            key, val = t[0], t[1]
            if val[selector] > last_time:
                best_key = key
                last_time = val[selector]
        return best_key

    def get_ys(self, y):
        '''Returns a list of all of the properties or types
        (pass as y).

        Args:
            y (string): 'property' | 'type'
        '''
        return list(set([val[y] for val in self.obj_dict.itervalues()
            if val[y] is not None]))

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
            # TODO(max): Is referring also considered changing?
            self.obj_dict[nlp_obj]['last-changed'] = now
