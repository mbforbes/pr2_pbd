#!/usr/bin/env python

# This is a pretty good solution (just pass the expanded version of
# ~/.config/google-chrome/Default/File System/002/t/00
# and it updates when the file instide (00000000) is changed.

# ROS imports first
import roslib
roslib.load_manifest('pr2_pbd_speech_recognition')
import rospy

# System
import os
import sys
import time
import logging

# 3rd party
from watchdog.observers import Observer
from watchdog.events import FileSystemEventHandler, LoggingEventHandler

# Local imports
from pr2_pbd_msgs.msg import RecognizedSpeech

class SpeechEventHandler(FileSystemEventHandler):
    def startup(self):
        self.pub = rospy.Publisher('gspeech/output', RecognizedSpeech)

    def on_modified(self, event):
        '''For existing users---file already exists. We kind of assume
        this as Google Chrome can cache the file in any number of
        directories.
        '''
        self.pub.publish(
            RecognizedSpeech(
                os.popen("tail -n 1 \"%s\"" % event.src_path).read().strip(),
                0.75))

if __name__ == "__main__":
    rospy.init_node('chromeASR')
    logging.basicConfig(level=logging.INFO,
                        format='%(asctime)s - %(message)s',
                        datefmt='%Y-%m-%d %H:%M:%S')
    path = "/home/mbforbes/.config/google-chrome/Default/File System/002/t/00"
    #path = sys.argv[1] if len(sys.argv) > 1 else '.'
    #event_handler = LoggingEventHandler()
    event_handler = SpeechEventHandler()
    event_handler.startup()
    observer = Observer()
    observer.schedule(event_handler, path, recursive=False)
    observer.start()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        observer.stop()
    observer.stop()
    observer.join()
