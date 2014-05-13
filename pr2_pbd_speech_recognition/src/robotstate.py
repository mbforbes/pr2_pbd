'''Tracks a specific set of robot state attributes for NLP processing.

Gets updates to robot experiment state and provides access to the data
in an interpretable format.
'''

__author__ = 'mbforbes'

# TODO(max): Do we need ROS stuff here?

# ROS imports come first
import roslib
roslib.load_manifest('pr2_pbd_speech_recognition')
import rospy

class RobotState:
    '''The class that provides access to the robot state.'''

    def __init__(self):
        pass

    def getState(self):
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

        No combination (4)
        ------------------
        - execution
        - microphone
        - motion
        - object

        Only one combination (1)
        ------------------------
        - command
            - last

        Multiple combinations (11)
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
        - pose
            - ''
            - object
        - step
            - all
            - last


        When something's not implemented:
        =================================
        'NULL'
        '''
        return {
            'lastChanged': self.getLastChanged(),
            'lastChangedByProperty': self.getLastChangedByProperty(),
            'lastChangedObjByType': self.getLastChangedObjByType(),
            'lastReferred': self.getLastReferred(),
            'lastReferredByProperty': self.getLastReferredByProperty(),
            'lastReferredByType': self.getLastReferredByType(),
            'stateByObj': self.getStateByObj()
        }

    def getLastChanged(self):
        '''The last object in the system that changed.

        returns string Obj
        '''
        # TODO(max): Implement.
        return 'NULL'

    def getLastChangedByProperty(self):
        '''For each property in {@link Obj#getProperties()} the last
        object that changed it status.

        returns {string: string} (Map<String, Obj>)
        '''
        # TODO(max): Figure out what the heck this means. Implement.
        return 'NULL'

    def getLastChangedObjByType(self):
        '''For each {@link Obj#getLabel()}, the last object that changed
        its state.

        returns {string: string} (Map<String, Obj>)
        '''
        return 'NULL'

    def getLastReferred(self):
        '''The last object in the system that was referred by the user.

        returns string (obj)
        '''
        # TODO(max): Implement.
        return 'NULL'

    def getLastReferredByProperty(self):
        '''For each property in {@link Obj#getProperties()} the last
        object referred to by the user.

        returns {string: string} (Map<String, Obj>)
        '''
        # TODO(max): Implement.
        return 'NULL'

    def getLastReferredByType(self):
        '''
        For each {@link Obj#getLabel()}, the last object referred to by
        the user.

        returns {string: string} (Map<String, Obj>)
        '''
        # TODO(max): Implement.
        return 'NULL'

    def getStateByObj(self):
        '''For each {@link Obj} the current state.

        returns {string: string} (Map<Obj, String>)
        '''
        # TODO(max): Implement.
        return 'NULL'
