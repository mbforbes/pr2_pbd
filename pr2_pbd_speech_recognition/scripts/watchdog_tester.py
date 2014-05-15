#!/usr/bin/env python

# This is a pretty good solution (just pass the expanded version of
# ~/.config/google-chrome/Default/File System/002/t/00
# and it updates when the file instide (00000000) is changed.

import os
import sys
import time
import logging
from watchdog.observers import Observer
from watchdog.events import FileSystemEventHandler, LoggingEventHandler


class SpeechEventHandler(FileSystemEventHandler):
    def on_modified(self, event):
        '''For existing users---file already exists. We kind of assume
        this as Google Chrome can cache the file in any number of
        directories.
        '''
        print os.popen("tail -n 1 \"%s\"" % event.src_path).read().strip()

if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO,
                        format='%(asctime)s - %(message)s',
                        datefmt='%Y-%m-%d %H:%M:%S')
    path = sys.argv[1] if len(sys.argv) > 1 else '.'
    #event_handler = LoggingEventHandler()
    event_handler = SpeechEventHandler()
    observer = Observer()
    observer.schedule(event_handler, path, recursive=False)
    observer.start()
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        observer.stop()
    observer.join()
