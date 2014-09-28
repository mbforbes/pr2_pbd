'''Convenience classes for communication.'''

__author__ = 'mbforbes'


# ######################################################################
# Imports
# ######################################################################

# Core ROS imports come first.
import roslib
roslib.load_manifest('pr2_pbd_interaction')
import rospy

# PbD (3rd party / local)
from response import Response
from pr2_social_gaze.msg import GazeGoal
from util import Logger

# ######################################################################
# Classes
# ######################################################################


class Feedback(object):
    '''Becuase 'Response' was already taken in normal PbD.'''

    def __init__(self, speech=None, gaze=None):
        '''
        Args:
            speech (str, optional): What to say. Defaults to None.
            gaze (int, optional): Head movement to do. Should be one of
                the constants defined in GazeAction.action. Defaults to
                None.
        '''
        self.speech = speech
        self.gaze = gaze

    def issue(self):
        '''Issues the Feedback (says and/or gazes).'''
        if self.speech is not None:
            Logger.L.fb(self.speech)
            Response.say(self.speech)
        if self.gaze is not None:
            Response.perform_gaze_action(self.gaze)


class FailureFeedback(Feedback):
    '''Shakes head and says whatever.'''

    def __init__(self, speech=None):
        '''
        Args:
            speech (str, optional): What to say. Defaults to None.
        '''
        super(FailureFeedback, self).__init__(speech, GazeGoal.SHAKE)
