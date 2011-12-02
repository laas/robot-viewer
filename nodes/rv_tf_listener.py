import roslib, os, sys
roslib.load_manifest('robotviewer')
import rospy
import robotviewer.client
import tf

def main():
    """
    """
    rospy.init_node("rv_tf_listener")
    server = robotviewer.client()

    tf_map = rospy.get_param("~tf_map",{})
    if isinstance(tf_map, str):
        tf_map = eval(tf_map)
    print tf_map
    for rv_name, ros_name in tf_map.items():
        elements = server.listElements()
        if rv_name not in elements:
            path = os.path.abspath(os.path.dirname(__file__))
            axes_fn = os.path.join(path, "..","src","models", "axes3.wrl")
            server.createElement('object', rv_name, axes_fn)
            server.setScale(rv_name, [.1, .1, .1])

    transformer = tf.TransformListener()
    def tf_listen():
        for rv_name, ros_name in tf_map.items():
            #rospy.loginfo("Listening to {0}".format(ros_name))

            try:
                now = rospy.Time.now()
                trans, rot = transformer.lookupTransform('world',ros_name,
                                                         rospy.Time())
            except tf.LookupException:
                rospy.logfatal("Lookup fail, transform {0}-{1}".format('/world', ros_name))
                continue
            except tf.ConnectivityException:
                rospy.logfatal("Not connect, transform {0}-{1}".format('/world', ros_name))
                continue

            except tf.ExtrapolationException:
                rospy.logfatal("Fail to extrapolate, transform {0}-{1}".format('/world', ros_name))
                continue

            T = tf.transformations.quaternion_matrix(rot)
            T[:3,3] = trans
            T = [ list(line) for line in T]
            server.updateElementConfig2(rv_name, T, [])
            rospy.loginfo("Sending {0} to {1}".format(T, rv_name))
            print trans

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        tf_listen()
        rate.sleep()


if __name__ == '__main__':
    main()
