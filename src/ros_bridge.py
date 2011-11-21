import roslib, os, sys
roslib.load_manifest('rospy')
roslib.load_manifest('dynamic_reconfigure')
import rospy
import numpy
import dynamic_reconfigure.client
def callback(config):
    rospy.loginfo("Config set to {int_param}, {double_param}, {str_param}, {bool_param}, {size}".format(**config))

class Bridge(object):
    def __init__(self, server):
        self.server = server
        rospy.init_node('robotviewer')
        #cfg = numpy.array(server.getElementConfig2('CAMERA_LU'))
        #client = dynamic_reconfigure.client.Client("dynamic_tutorials", timeout=30, config_callback=callback)
