''' Robot speech'''

# Mandatory ROS imports.
import roslib
roslib.load_manifest('pr2_pbd_interaction')
import rospy

# Builtins.
import time
from threading import Thread, Lock

# ROS.
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from std_msgs.msg import Header, ColorRGBA


class RobotSpeech:
    ''' The robot's speech responses '''

    TEST_RESPONSE = 'Microphone working.'
    SKILL_CREATED = 'Created skill.'
    RIGHT_ARM_RELEASED = 'Right arm released.'
    RIGHT_ARM_HOLDING = 'Right arm holding.'
    RIGHT_HAND_OPENING = 'Opening right hand.'
    RIGHT_HAND_CLOSING = 'Closing right hand.'
    LEFT_ARM_RELEASED = 'Left arm released.'
    LEFT_ARM_HOLDING = 'Left arm holding.'
    LEFT_HAND_OPENING = 'Opening left hand.'
    LEFT_HAND_CLOSING = 'Closing left hand.'
    STEP_RECORDED = 'Pose saved.'
    POSE_DELETED = 'Last pose deleted.'
    POSE_RESUMED = 'Pose resumed.'
    DELETED_SKILL = 'Deleted skill.'
    START_EXECUTION = 'Starting execution of skill.'
    EXECUTION_ENDED = 'Execution ended.'
    SWITCH_SKILL = 'Switched to skill.'
    SKILL_EMPTY = 'Skill has no poses to delete.'
    EXECUTION_ERROR_NOIK = 'Cannot execute skill.'
    EXECUTION_ERROR_NOPOSES = 'Not enough poses in skill.'
    ERROR_NEXT_SKILL = 'No skills after skill.'
    ERROR_PREV_SKILL = 'No skills before skill.'
    ERROR_NO_SKILLS = 'No skills created yet.'
    ERROR_NOTHING_TO_UNDO = 'There is nothing to undo.'
    ERROR_NO_EXECUTION = 'No executions in progress.'
    EXECUTION_PREEMPTED = 'Stopping execution.'
    RIGHT_HAND_ALREADY_OPEN = 'Right hand is already open.'
    LEFT_HAND_ALREADY_OPEN = 'Left hand is already open.'
    RIGHT_HAND_ALREADY_CLOSED = 'Right hand is already closed.'
    LEFT_HAND_ALREADY_CLOSED = 'Left hand is already closed.'
    RIGHT_ARM_ALREADY_HOLDING = 'Right arm holding.'
    LEFT_ARM_ALREADY_HOLDING = 'Left arm holding.'
    RIGHT_ARM_ALREADY_RELEASED = 'Right arm released.'
    LEFT_ARM_ALREADY_RELEASED = 'Left arm released.'
    SKILL_CLEARED = 'All poses deleted, skill cleared.'
    LAST_POSE_DELETED = 'Last pose deleted.'
    ALL_POSES_RESUMED = 'All poses resumed.'
    START_STATE_RECORDED = 'Object poses recorded.'
    OBJECT_NOT_DETECTED = 'No objects were detected.'
    ACTION_SAVED = 'Saved skill '
    ALREADY_EDITING = 'Already in editing mode.'
    SWITCH_TO_EDIT_MODE = 'Switched to edit mode.'
    ERROR_NOT_IN_EDIT = ' has been saved. Say, edit skill, to make changes.'
    ACTION_ALREADY_STARTED = ('Skill already started. ' +
                             'Say, delete all steps, to start over.')
    ALREADY_RECORDING_MOTION = 'Already recording motion.'
    STARTED_RECORDING_MOTION = 'Started recording motion.'
    STOPPED_RECORDING_MOTION = 'Stopped recording motion.'
    MOTION_NOT_RECORDING = 'Not currently recording motion.'
    STOPPING_EXECUTION = 'Execution stopped.'
    ERROR_UNRECOGNIZED = 'Please rephrase your request.'

    # Waiting period between when we say things. SoundClient.say()
    # neither blocks nor gives a call-back, so this heuristic appears to
    # be the best we can do.
    #
    # This is emperically set by timing "Right hand is already open."
    SAY_WAIT_PERIOD_SECONDS = 2.3

    def __init__(self):
        #self.sound_publisher = rospy.Publisher('robotsound', SoundRequest)
        self.soundhandle = SoundClient()
        self.marker_publisher = rospy.Publisher('visualization_marker', Marker)

        self._last_said = 0
        # NOTE(max): We don't lock when we say something as say() is
        # non-blocking. Instead, we have some default waiting period,
        # and use the lock to avoid threading bugs.
        self._wait_say_lock = Lock()

    def say(self, text, is_using_sounds=False):
        ''' Send a TTS command'''
        if (not is_using_sounds):
            # NOTE(max): This old way isn't saying the text (just beeps)
            # self.sound_publisher.publish(
                # SoundRequest(command=SoundRequest.SAY, arg=text))
            say_thread = Thread(group=None, target=self.async_say,
                args=[text], name='async_say_thread')
            say_thread.start()

    def async_say(self, text):
        self._wait_say_lock.acquire()
        while True:
            time_delta = time.time() - self._last_said
            if time_delta < RobotSpeech.SAY_WAIT_PERIOD_SECONDS:
                time.sleep(RobotSpeech.SAY_WAIT_PERIOD_SECONDS - time_delta)
            else:
                rospy.loginfo('Trying to say:' + text)
                self.soundhandle.say(text)
                self.say_in_rviz(text)
                self._last_said = time.time()
                break
        self._wait_say_lock.release()

    def say_in_rviz(self, text):
        ''' Visualizes the text that is uttered by the robot in rviz'''
        marker = Marker(type=Marker.TEXT_VIEW_FACING, id=1000,
                   lifetime=rospy.Duration(1.5),
                   pose=Pose(Point(0.5, 0.5, 1.45), Quaternion(0, 0, 0, 1)),
                   scale=Vector3(0.06, 0.06, 0.06),
                   header=Header(frame_id='base_link'),
                   color=ColorRGBA(0.0, 1.0, 0.0, 0.8), text=text)
        self.marker_publisher.publish(marker)
