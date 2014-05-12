#!/usr/bin/env python

# ROS imports come first
import roslib
roslib.load_manifest('pr2_pbd_speech_recognition')
import rospy

# 3rd party
import requests

# Local imports
from robotstate import RobotState
from std_msgs.msg import String
from pr2_pbd_speech_recognition.msg import Command
from pr2_pbd_speech_recognition.msg import RecognizedSpeech

# Constants
NLP_SERVER = 'http://localhost'
NLP_PORT = 10001

class CommandRecognizer:
    '''This class is responsible for receiving recognized speech,
    transforming it into a system command, and publishing that
    command.'''

    def __init__(self):
        # Construct RobotState so that it can track / return state info
        # for smarter NLP processing.
        self.robotState = RobotState()

        # Subscribe to pocketsphinx (if it exists).
        rospy.Subscriber('recognizer/output', String, self.receiveSphinxData)

        # Subscribe to gspeech (if it exists).
        rospy.Subscriber('gspeech/output', RecognizedSpeech,
            self.receiveGspeechData)

        # Here's what we'll give back to the system.
        self.commandOutput = rospy.Publisher('recognized_command', Command)
        self.allCommands = [
            Command.TEST_MICROPHONE,
            Command.RELAX_RIGHT_ARM,
            Command.RELAX_LEFT_ARM,
            Command.OPEN_RIGHT_HAND,
            Command.OPEN_LEFT_HAND,
            Command.CLOSE_RIGHT_HAND,
            Command.CLOSE_LEFT_HAND,
            Command.STOP_EXECUTION,
            Command.UNDO,
            Command.DELETE_ALL_STEPS,
            Command.DELETE_LAST_STEP,
            Command.FREEZE_RIGHT_ARM,
            Command.FREEZE_LEFT_ARM,
            Command.RECORD_OBJECT_POSE,
            Command.CREATE_NEW_ACTION,
            Command.EXECUTE_ACTION,
            Command.NEXT_ACTION,
            Command.PREV_ACTION,
            Command.SAVE_ACTION,
            Command.EDIT_ACTION,
            Command.SAVE_POSE,
            Command.START_RECORDING_MOTION,
            Command.STOP_RECORDING_MOTION
        ]

    def receiveSphinxData(self, data):
        '''Sphinx is trained only with the commands.'''
        print data
        recognizedStr = data.data
        recognizedCommand = Command.UNRECOGNIZED

        # NOTE(max): Testing full pocketsphinx capabilities.
        rospy.loginfo('Sphinx got: ' + recognizedStr)

        for commandStr in self.allCommands:
            if (recognizedStr == commandStr):
                recognizedCommand = commandStr

        #rospy.loginfo('Received command:' + recognizedCommand)
        command = Command()
        command.command = recognizedCommand
        self.commandOutput.publish(command)

    def receiveGspeechData(self, recognizedSpeech):
        '''Gspeech should recognize arbitrary speech. We'll then send
        this to the NLP semantic-parsing pipeline.'''
        print '[NLP] Recognizer heard:', recognizedSpeech.text
        params = {
            'sentence': recognizedSpeech.text,
            'state': self.robotState.getState()
        }
        print '[NLP] Robot state:', params
        try:
            response = requests.get('%s:%d' % (NLP_SERVER, NLP_PORT),
                params=params).text.strip()
            # Make sure we got something non-empty back.
            print '[NLP] Semantically parsed:', response
            if len(response) > 0 and len(eval(response)) > 0:
                cmd = eval(response)[0]
                # Do sanity checking.
                if cmd in self.allCommands:
                    recognizedCommand = cmd
                else:
                    recognizedCommand = Command.UNRECOGNIZED
                # Send it off
                self.commandOutput.publish(Command(recognizedCommand))
        except requests.exceptions.ConnectionError as e:
            rospy.logwarn('NLP server not running. Dropping recognized ' +
                'speech: ' + recognizedSpeech.text)

if __name__ == '__main__':
    rospy.init_node('command_recognizer')
    crec = CommandRecognizer()
    rospy.spin()
