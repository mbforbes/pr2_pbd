#!/usr/bin/env python
'''A simple ROS node so that we can launch a web server to allow for
Javascript shenanigans (with no server, access to local file system is
restricted).

Thanks to http://www.linuxjournal.com/content/tech-tip-really-simple-
http-server-python for the base of the code.
'''

__author__ = 'mbforbes'

# ROS imports come first.
import roslib
roslib.load_manifest('pr2_pbd_http')
import rospy

# Then built-ins.
import os
import sys
from BaseHTTPServer import HTTPServer
from SimpleHTTPServer import SimpleHTTPRequestHandler

def serve():
    '''Just serve requests forever.'''
    protocol = "HTTP/1.0"

    # TODO(max): Add command line options for:
    # - directory
    # - port
    server_address = ('127.0.0.1', 8000)

    SimpleHTTPRequestHandler.protocol_version = protocol
    httpd = HTTPServer(server_address, SimpleHTTPRequestHandler)

    sa = httpd.socket.getsockname()
    os.chdir(os.path.expanduser('~/rosbuild_ws/pr2_pbd/pr2_pbd_http'))
    print "Serving HTTP on", sa[0], "port", sa[1], "..."
    httpd.serve_forever()

if __name__ == '__main__':
    rospy.init_node('python_web_server')
    try:
        serve()
    except KeyboardInterrupt:
        sys.exit(0)
