#!/usr/bin/env python

# ros package for speech recognition using Google Speech API
#
# written by achuwilson
# 30-06-2012 , 3.00pm
# achu@achuwilson.in
#
# modified by Zhongyue (Jerry) Zhang
# 3/19/2014
# jerryzh.cn@gmail.com
#
# modified by Andrzej Pronobis
# 4/01/2014
# a.pronobis@gmail.com
#
# modified by Maxwell Forbes
# 5/9/2014
# m.b.forbes@gmail.com

# ROS imports first
import roslib
roslib.load_manifest('pr2_pbd_speech_recognition')
import rospy

# System imports
import json
import os
import shlex
import subprocess
import sys

# Local imports
from pr2_pbd_msgs.msg import RecognizedSpeech

# Constants
# ----------------------------------------------------------------------

# URL / comamnd constants
KEY = 'AIzaSyCnl6MRydhw_5fLXIdASxkLJzcJh5iX0M4'
API_URL = 'http://www.google.com/speech-api/v2/recognize?lang=en-us&key='
FILENAME_BASE = 'recording.flac'
FILENAME_NORMED = 'recordingNorm.flac'
HEADER = 'Content-Type: audio/x-flac; rate=16000" -O - '

# Audio constants.
# SILENCE_END: after we start hearing sound, how long of a pause we need
# before we consider the utterance 'finished' and send it off to the
# speech recognizer.
SILENCE_END = '0.05' # was 2.0 originally

# DEFAULT_CONFIDENCE: Sometimes Google's API won't return a confidence
# for its top guess. This is the default that we use.
DEFAULT_CONFIDENCE = 0.75

# The string and confidence we use when we got an error or nothing
# recognized back from the speech recognizer.
BAD_STR, BAD_CONF = '', 0.0

# The commands themselves.
# Record speech.
cmd_record = ('sox -q -r 16000 -t alsa default ' +
    FILENAME_BASE +
    ' silence 1 0.1 0.8% ' + SILENCE_END + ' 1.5 1%')
#   ' silence 1 0.1 0.8% 2 1.5 1%') # orig

# Send the data to Google speech recognition service.
cmd_recognize = ('wget -q -U "Mozilla/5.0" --post-file ' +
    FILENAME_NORMED +
    ' --header="' + HEADER + '"' +
    API_URL + KEY + '"')

# Normalize the audio (it seems to improve recognition)
cmd_norm='sox ' + FILENAME_BASE + ' ' + FILENAME_NORMED + ' gain -n -3'

# Functions
# ----------------------------------------------------------------------

def speech():
    '''Runs in a loop to recognize speech and publish the result.'''
    rospy.init_node('gspeech')
    pub = rospy.Publisher('gspeech/output', RecognizedSpeech)

    args_recognize = shlex.split(cmd_recognize)
    while not rospy.is_shutdown():
        print '[ASR] Listening...'
        os.system(cmd_record)
        print '[ASR] Normalizing...'
        os.system(cmd_norm)
        print '[ASR] Recognizing...'
        output, error = subprocess.Popen(
            args_recognize,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE).communicate()

        # my own processing
        if not error:
            if len(output.splitlines()) < 2:
                # Didn't get a good result.
                trans, conf = BAD_STR, BAD_CONF
            else:
                js = json.loads(output.splitlines()[1])
                idx = js['result_index']
                top = js['result'][0]['alternative'][idx]
                if top.has_key('confidence'):
                    conf = top['confidence']
                else:
                    conf = DEFAULT_CONFIDENCE
                trans = str(top['transcript'])
            # Publish.
            pub.publish(RecognizedSpeech(trans, conf))
        else:
            print '[ASR] Recognizer got error:', error

if __name__ == '__main__':
    rospy.init_node('gspeech')
    try:
        speech()
    except rospy.ROSInterruptException: pass
    except KeyboardInterrupt:
        sys.exit(1)
