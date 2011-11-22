import roslib, os, sys
roslib.load_manifest('robotviewer')
import rospy
import numpy
# import dynamic_reconfigure.client
from sensor_msgs.msg import JointState, Image, CameraInfo
from std_msgs.msg import Header

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

    def caminfo_cb(self, name):
        def cb(data):
            if time.time() - self.cam_update_t[name] < 0.5:
                return
            width = data.width
            height = data.height
            fx = data.P[0]
            cx = data.P[2]
            fy = data.P[5]
            cy = data.P[6]
            self.camera_dict[name].set_opencv_params(width, height, fx, fy, cx, cy)
            self.cam_update_t[name] = time.time()


        return cb


    def flipH(self, data, height, width):
        data = [[ data[width*i+j] for j in range(width)] for i in reversed(range(height))]
        return data

    def publish(self):
        win = glutGetWindow()
        cam = self.server.windows[win].camera
        if cam.pixels:
            im = Image()
            im.height = cam.height
            im.width = cam.width
            image = PIL.Image.fromstring(mode="RGB",
                                         size=(cam.width, cam.height), data=cam.pixels)
            image = image.transpose(PIL.Image.FLIP_TOP_BOTTOM)

            im.data = image.tostring()
            #im.data = cam.pixels
            im.encoding = "rgb8"
            im.is_bigendian = 0
            im.step = im.width*3 # 3 channels
            im.header.frame_id = cam.name
            im.header.stamp = rospy.Time(cam.draw_t)
            im.header.seq = cam.frame_seq
            self.image_pubs[cam.name].publish(im)


            caminfo = CameraInfo()
            caminfo.header = im.header
            caminfo.height = cam.height
            caminfo.width = cam.width
            caminfo.D = 5*[0.]
            caminfo.K = sum([list(r) for r in cam.K],[])
            caminfo.P = sum([list(r) for r in cam.P],[])
            caminfo.R = sum([list(r) for r in cam.R],[])

            self.camera_info_pubs[cam.name].publish(caminfo)
            self.cam_update_t[cam.name] = cam.draw_t



    def __init__(self, server):
        self.server = server
        rospy.init_node('robotviewer')

        jointstates = rospy.get_param("~jointstates",{})
        if isinstance(jointstates, str):
            jointstates = eval(jointstates)
        cam_map = rospy.get_param("~cam_map",{})

        if isinstance(cam_map, str):
            cam_map = eval(cam_map)

        for rvname, topic in jointstates.items():
            rospy.Subscriber(topic, JointState, self.state_cb(rvname))

        for rvcam, realcam in cam_map.items():
            rospy.Subscriber(realcam+"/camera_info", CameraInfo, self.caminfo_cb(rvcam))


        self.image_pubs = {}
        self.camera_info_pubs = {}
        self.cameras = self.server.cameras
        self.camera_dict = dict((cam.name, cam) for cam in self.cameras)
        for cam in self.cameras:
            self.image_pubs[cam.name] = rospy.Publisher(cam.name + "/image_raw", Image)
            self.camera_info_pubs[cam.name] = rospy.Publisher(cam.name + "/camera_info",
                                                              CameraInfo)
            self.cam_update_t[cam.name] = 0.

        server.post_draw = self.publish
