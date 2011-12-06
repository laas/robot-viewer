import roslib, os, sys
roslib.load_manifest('robotviewer')
import rospy
import numpy
# import dynamic_reconfigure.client
from sensor_msgs.msg import JointState, Image, CameraInfo
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped

from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *
from OpenGL.GLX import *
from OpenGL.GL.EXT.framebuffer_object import *
import tf
import time
import PIL.Image

import yaml
import cv
from cv_bridge import CvBridge, CvBridgeError
from camera import Camera
import threading

GL_TO_CV = numpy.array([
        [1, 0, 0, 0],
        [0, -1, 0, 0],
        [0, 0, -1, 0],
        [0, 0, 0, 1],
        ])

class Bridge(object):
    cam_update_t = {}

    def __init__(self, server):
        self.server = server
        rospy.init_node('robotviewer', anonymous = True)
        self.stamp = rospy.Time.now()
        self.tf_br = tf.TransformBroadcaster()
        self.transformer = tf.TransformListener()
        self.bridge = CvBridge()
        self.image_pubs = {}
        self.camera_info_pubs = {}
        self.cameras = self.server.cameras
        self.camera_dict = dict((cam.name, cam) for cam in self.cameras)
        self.count = 0
        for cam in self.server.cameras:
            self.image_pubs[cam.name] = rospy.Publisher(cam.name +
                                                        "/image_raw", Image)
            self.camera_info_pubs[cam.name] = rospy.Publisher(cam.name +
                                                              "/camera_info",
                                                              CameraInfo)
            self.cam_update_t[cam.name] = 0.

        jointstates = rospy.get_param("~jointstates",{})
        if isinstance(jointstates, str):
            jointstates = eval(jointstates)

        cam_calibs = rospy.get_param("~cam_calibs",{})
        if isinstance(cam_calibs, str):
            cam_calibs = eval(cam_calibs)

        for name, calib_file in cam_calibs.items():
            rospy.loginfo("Calibrating {0}".format(name))
            self.calibrate(self.camera_dict[name], calib_file)

        for rvname, topic in jointstates.items():
            rospy.Subscriber(topic, JointState, self.state_cb(rvname))

        self.pose_pubs = {}
        self.state_pubs = {}
        for robot in self.server.robots:
            rname = robot.name
            topic_name = "state/" + rname
            self.state_pubs[robot] = rospy.Publisher(topic_name, JointState)
            for obj in robot.joint_list + robot.cam_list:
                topic_name = "pose/" + rname + "/" + obj.name
                self.pose_pubs[obj] = rospy.Publisher(topic_name,
                                                      PoseStamped)



        server.idle_cbs.append (self.spin)

        # class TfThread(threading.Thread):
        #     def __init__(self, bridge):
        #         threading.Thread.__init__(self)
        #         self.b = bridge

        #     def run(self):
        #         while not rospy.is_shutdown():
        #             rospy.sleep(0.05)
        #             self.b.tf_listen()


        # t = TfThread(self)
        # t.start()

    def state_cb(self, objname):
        def cb(data):
            state = data.position
            self.server.updateElementConfig2(objname, [], state)
            self.server.elements[objname].stamp = data.header.stamp
            self.tf_br.sendTransform

        return cb


    def flipH(self, data, height, width):
        data = [[ data[width*i+j] for j in range(width)] for i in reversed(range(height))]
        return data

    def fetch_image(self, cam):
        cam.simulate()

        if not cam.pixels:
            return None, None
        cv_img = cv.CreateImageHeader((cam.width , cam.height), cv.IPL_DEPTH_8U, 3)
        cv.SetData(cv_img, cam.pixels, cam.width*3)
        cv.ConvertImage(cv_img, cv_img, cv.CV_CVTIMG_FLIP)
        im = self.bridge.cv_to_imgmsg(cv_img, "rgb8")

        caminfo = CameraInfo()
        caminfo.header = im.header
        caminfo.height = cam.height
        caminfo.width = cam.width
        caminfo.D = 5*[0.]
        caminfo.K = sum([list(r) for r in cam.K],[])
        caminfo.P = sum([list(r) for r in cam.P],[])
        caminfo.R = sum([list(r) for r in cam.R],[])

        return im, caminfo


    def tf_broadcast(self):
        for robot in self.server.robots:
            rname = robot.name
            state = JointState()
            if robot.stamp:
                state.header.stamp = robot.stamp
            else:
                state.header.stamp = self.stamp

            state.position = robot.get_config()[6:]
            self.state_pubs[robot].publish(state)
            for obj in robot.moving_joint_list + robot.cam_list:
                name = obj.name
                localT = obj.localTransformation
                T = obj.T
                if obj in robot.cam_list:
                    T = numpy.dot(obj.T, GL_TO_CV)
                    localT = numpy.dot(localT, GL_TO_CV)
                quat = tf.transformations.quaternion_from_matrix(T)
                local_quat = tf.transformations.quaternion_from_matrix(localT)
                if obj == robot.root_joint or obj.parent == None:
                    parent_name = "world"
                else:
                    parent_name = "{0}/{1}".format(rname, obj.parent.name)
                self.tf_br.sendTransform(localT[:3,3],
                                         local_quat,
                                         state.header.stamp,
                                         "{0}/{1}".format(rname, name),
                                         parent_name,
                                         )
                pose_msg = PoseStamped()
                pose_msg.header.stamp = state.header.stamp
                pose_msg.pose.position.x = T[0][3]
                pose_msg.pose.position.y = T[1][3]
                pose_msg.pose.position.z = T[2][3]
                pose_msg.pose.orientation.x = quat[0]
                pose_msg.pose.orientation.y = quat[1]
                pose_msg.pose.orientation.z = quat[2]
                pose_msg.pose.orientation.w = quat[3]
                self.pose_pubs[obj].publish(pose_msg)


    def cam_publish(self):
        for cam in self.server.cameras:
            image_pub = self.image_pubs[cam.name]
            caminfo_pub = self.camera_info_pubs[cam.name]


            if ( image_pub.get_num_connections() == 0
                 and caminfo_pub.get_num_connections() == 0):
                continue

            im, caminfo = self.fetch_image(cam)

            if im == None:
                continue

            im.header.stamp = self.stamp
            caminfo.header.stamp = self.stamp


            image_pub.publish(im)
            caminfo_pub.publish(caminfo)

            self.cam_update_t[cam.name] = cam.draw_t


    def spin(self):
        self.stamp = rospy.Time.now()
        self.cam_publish()
        self.tf_broadcast()

    def calibrate(self, cam, cf):
        params = yaml.load(open(cf).read())
        width = params['image_width']
        height = params['image_height']
        P = params['projection_matrix']['data']
        fx = P[0]
        cx = P[2]
        fy = P[5]
        cy = P[6]
        R = params['rectification_matrix']['data']
        cam.R[0,:] = R[:3]
        cam.R[1,:] = R[3:6]
        cam.R[2,:] = R[6:]
        cam.set_opencv_params(width, height, fx, fy, cx, cy)
        rospy.loginfo("Setting OpenCV params {0}".format((width, height, fx,
                                                          fy, cx, cy))
                      )

