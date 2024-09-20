import rospy
import tf

import time

from tf import transformations as t
from tf import LookupException
from tf import ExtrapolationException

"""
This node properly connects tf trees created by slam_toolbox and lio-sam by matching up the /velodyne frame on the robot with lio-sam`s /lidar_link.

The node is required, since tf can only deal with tree-shaped graphs. Two base_links matched up (as we are doing here) can not be resolved.

"""

def waitForLookup(targetFrame, sourceFrame, listener, maxTries=2000):
    (trans, rot) = (None, None)
    for i in range(maxTries):
        try:
            (trans, rot) = listener.lookupTransform(targetFrame, sourceFrame, rospy.Time(0))
        except (LookupException, ExtrapolationException):
            rospy.loginfo("Could not lookup transform. Will keep trying {} times".format(maxTries-i))
            time.sleep(1)
        else:
            break
    return (trans, rot)

def main():
    rospy.init_node("map_connector")
    listener = tf.TransformListener()
    broadcaster = tf.TransformBroadcaster()
    rate = rospy.Rate(5)


    while not rospy.is_shutdown():
        # Lookup Transform from main map to main base_link (2D pipeline serves as an anchor)
        #(trans, rot) = waitForLookup("map", "base_link", listener)
        #
        #broadcaster.sendTransform(
        #    trans,
        #    rot,
        #    rospy.Time.now(),
        #    "base_link2",
        #    "map2"
        #)
        #
        # Lookup transform from separate base_link to separate map (= inverse transform)
        (trans, rot) = waitForLookup("lidar_link", "liosam_map", listener)
        #
        ## Sometimes, tf does not want to invert the transform properly. In this case, swap target frames in the line above an uncomment the following lines:
        # Invert transform: https://answers.ros.org/question/229329/what-is-the-right-way-to-inverse-a-transform-in-python/
        #transform = t.concatenate_matrices(t.translation_matrix(trans), t.quaternion_matrix(rot))
        #inversed_transform = t.inverse_matrix(transform)
        #trans = t.translation_from_matrix(inversed_transform)
        #rot = t.quaternion_from_matrix(inversed_transform)
        #
        # Publish the transform but going from base_link to map2 to set both base_links to the same pose
        broadcaster.sendTransform(
            trans,
            rot,
            rospy.Time.now(),
            "liosam_map",
            "velodyne"
        )
        rate.sleep()

if __name__=="__main__":
    main()