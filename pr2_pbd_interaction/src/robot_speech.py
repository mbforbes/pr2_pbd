'''Robot speech: what the robot actually says.'''

# ######################################################################
# Imports
# ######################################################################

# Core ROS imports come first.
import roslib
roslib.load_manifest('pr2_pbd_interaction')
import rospy

# ROS builtins
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from std_msgs.msg import Header, ColorRGBA
from visualization_msgs.msg import Marker

# ROS 3rd party
from sound_play.libsoundplay import SoundClient
from sound_play.msg import SoundRequest


# ######################################################################
# Module level constants
# ######################################################################

# ROS topics
TOPIC_SPEECH = 'robotsound'
TOPIC_MARKER = 'visualization_marker'

# Constants for the Rviz display of "robot spoken" text.
TEXT_MARKER_ID = 1000
TEXT_MARKER_LIFETIME = rospy.Duration(1.5)
TEXT_POSITION = Point(0.5, 0.5, 1.45)
TEXT_ORIENTATION = Quaternion(0, 0, 0, 1)
TEXT_POSE = Pose(TEXT_POSITION, TEXT_ORIENTATION)
TEXT_SCALE = Vector3(0.06, 0.06, 0.06)
TEXT_COLOR = ColorRGBA(0.0, 1.0, 0.0, 0.8)  # green

# TODO(mbforbes): This should be refactored so it's in only one module.
BASE_LINK = 'base_link'


# ######################################################################
# Classes
# ######################################################################

class RobotSpeech:
    '''An instance is a controller for robot speech responses, realized
    both in spoken speech and visualized as text (e.g. in Rviz).

    There need only be one instance of this class.
    '''
    # General
    TEST_RESPONSE = 'Microphone working.'

    # Creating/moving around actions
    SKILL_CREATED = 'Created action'
    SWITCH_SKILL = 'Switched to action'
    ERROR_NEXT_SKILL = 'No actions after action'
    ERROR_PREV_SKILL = 'No actions before action'
    ERROR_NO_SKILLS = 'No actions created yet.'

    # Relax / freeze arms
    RIGHT_ARM_RELEASED = 'Right arm relaxed'
    RIGHT_ARM_HOLDING = 'Right arm frozen'
    LEFT_ARM_RELEASED = 'Left arm relaxed'
    LEFT_ARM_HOLDING = 'Left arm frozen'
    RIGHT_ARM_ALREADY_HOLDING = 'Right arm is already frozen.'
    RIGHT_ARM_ALREADY_RELEASED = 'Right arm is already relaxed.'
    LEFT_ARM_ALREADY_HOLDING = 'Left arm is already frozen.'
    LEFT_ARM_ALREADY_RELEASED = 'Left arm is already relaxed.'

    # Open/close hands
    RIGHT_HAND_OPENING = 'Opening right hand.'
    RIGHT_HAND_CLOSING = 'Closing right hand.'
    LEFT_HAND_OPENING = 'Opening left hand.'
    LEFT_HAND_CLOSING = 'Closing left hand.'
    RIGHT_HAND_ALREADY_OPEN = 'Right hand is already open.'
    RIGHT_HAND_ALREADY_CLOSED = 'Right hand is already closed.'
    LEFT_HAND_ALREADY_OPEN = 'Left hand is already open.'
    LEFT_HAND_ALREADY_CLOSED = 'Left hand is already closed.'

    # Recording object poses
    START_STATE_RECORDED = 'Start state recorded.'
    OBJECT_NOT_DETECTED = 'No objects were detected.'

    # Mucking with steps (poses).
    STEP_RECORDED = 'Pose saved.'
    SKILL_EMPTY = 'Action has no poses to delete.'
    LAST_POSE_DELETED = 'Last pose deleted.'
    SKILL_CLEARED = 'All poses deleted.'

    # Executing
    START_EXECUTION = 'Starting execution of action'
    EXECUTION_ENDED = 'Execution ended.'
    ERROR_NO_EXECUTION = 'No executions in progress.'
    EXECUTION_PREEMPTED = 'Stopping execution.'
    STOPPING_EXECUTION = 'Execution stopped.'
    EXECUTION_ERROR_NOIK = 'Cannot execute action'
    EXECUTION_ERROR_NOPOSES = 'Not enough poses in action'

    # Trajectories
    STARTED_RECORDING_MOTION = 'Started recording motion.'
    STOPPED_RECORDING_MOTION = 'Stopped recording motion.'
    MOTION_NOT_RECORDING = 'Not currently recording motion.'
    ALREADY_RECORDING_MOTION = 'Already recording motion.'

    def __init__(self):
        # The speech publisher doesn't actually make speech happen (the
        # SoundClient does that), but it lets us track speech in ROS
        # (like in the GUI and tests). It might be useful to see how
        # SoundClient does this and switch the GUI/tests to this (if
        # it's different).
        self.speech_publisher = rospy.Publisher(TOPIC_SPEECH, SoundRequest)
        self.marker_publisher = rospy.Publisher(TOPIC_MARKER, Marker)
        self.soundhandle = SoundClient()

    def say(self, text):
        '''Send a TTS (text to speech) command.

        This will cause the robot to verbalize text if not
        is_using_sounds. It will always display text in Rviz.

        Args:
            text (str): The speech to say / vizualize.
            is_using_sounds (bool): Whether the robot is beeping and
                booping (if True), which determines whether to actually
                speak the words (only if False).
        '''
        # ROS param (see backend launch file) dictates whether we speak.
        if rospy.has_param('robot_speech') and rospy.get_param('robot_speech'):
            # Again, this does nothing but allow us to track speech.
            self.speech_publisher.publish(SoundRequest(
                command=SoundRequest.SAY, arg=text))

            # This really says it.
            self.soundhandle.say(text)

        # We always display the text in RViz.
        self.say_in_rviz(text)

    def say_in_rviz(self, text):
        ''' Displays text in Rviz, in a position to imply the robot said
        it.

        Args:
            text (str) The speech to visualize.
        '''
        marker = Marker(
            type=Marker.TEXT_VIEW_FACING,
            id=TEXT_MARKER_ID,
            lifetime=TEXT_MARKER_LIFETIME,
            pose=TEXT_POSE,
            scale=TEXT_SCALE,
            header=Header(frame_id=BASE_LINK),
            color=TEXT_COLOR,
            text=text
        )
        self.marker_publisher.publish(marker)
