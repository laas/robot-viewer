import roslib, os, sys
roslib.load_manifest('robotviewer')
import rospy
import numpy
# import dynamic_reconfigure.client
from sensor_msgs.msg import JointState, Image

from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *
from OpenGL.GLX import *
from OpenGL.GL.EXT.framebuffer_object import *

import time
import PIL

class Bridge(object):
    cam_update_t = {}

    def state_cb(self, objname):
        def cb(data):
            state = data.position
            self.server.updateElementConfig2(objname, [], state)

        return cb

    def publish(self):
        win = glutGetWindow()
        cam = self.server.windows[win].camera
        if cam.pixels:
            im = Image()
            im.data = cam.pixels
            im.height = cam.height
            im.width = cam.width
            im.encoding = "rgb8"

                #im.step = im.height
            self.publishers[cam.name].publish(im)
            self.cam_update_t[cam.name] = cam.draw_t

    def __init__(self, server):
        self.server = server
        rospy.init_node('robotviewer')
        jointstates = rospy.get_param("~jointstates",{})

        for rvname, topic in jointstates.items():
            rospy.Subscriber(topic, JointState, self.state_cb(rvname))

        self.publishers = {}
        self.cameras = self.server.cameras

        for cam in self.cameras:
            self.publishers[cam.name] = rospy.Publisher(cam.name, Image)
            self.cam_update_t[cam.name] = 0.

        server.post_draw = self.publish
