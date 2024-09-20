#!/usr/bin/env python
"""Used to send a come home goal for mbf, converts goal in map frame if different frame given"""

import rospy
import tf2_ros
import tf2_py as tf2

from dynamic_reconfigure.server import Server

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped

from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_pose

from std_srvs.srv import Trigger
from std_srvs.srv import TriggerResponse

from comehome_pose_tf2.cfg import LookupTransformConfig


class ComehomePoseTF2:
    """Manages the given home position and waits for the service call to start driving home.
    Handles the navigation goals given by the operator via rviz. Should the given position not
    originate from the map frame (current frame in rviz) it will transforms the given position into
    the map frame. MBF needs the position in the map_frame otherwise an error will be returned.
    """

    def __init__(self):
        self.config = None
        self.home_pose = None
        self.home = False

        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10))
        self.tl = tf2_ros.TransformListener(self.tf_buffer)

        self.pub_new_goal = rospy.Publisher(
            "/move_base_simple/goal", PoseStamped, queue_size=1
        )

        self.sub_goal = rospy.Subscriber("/mux/goal", PoseStamped, self.goal_callback)
        self.sub_home = rospy.Subscriber(
            "/home_goal", PoseWithCovarianceStamped, self.home_callback
        )

        self.srv_home = rospy.Service(
            "/Comehome_GoalPoseTF/drive_home", Trigger, self.home_server
        )

        self.dr_server = Server(LookupTransformConfig, self.dr_callback)

    def dr_callback(self, config, _):
        """Dynamic reconfigure callback
        :param config: Object containing the configurations defined via dyn. reconf
        :param _: Level param not used
        :return: Object containing the configurations defined via dyn. reconf
        :todo: check if return can be removed
        """
        self.config = config
        return self.config

    def goal_callback(self, msg):
        """Transforms the goal position from the map coordinate frame to the
        robot coordinate frame and publishes it to /move_base_simple/goal
        :param msg: The current goal in map / robot tf frame
        :return: None
        """
        if self.config.enable is not True:
            return

        look_up_time = rospy.Time()
        if self.home is False:
            source_frame = (
                msg.header.frame_id
                if self.config.source_frame == ""
                else self.config.source_frame
            )
        else:
            self.home = False
            source_frame = msg.header.frame_id
        map_frame = self.config.map_frame

        if source_frame == map_frame:
            rospy.loginfo("Source frame and map frame identical -> no transformation")
            self.pub_new_goal.publish(msg)
            return

        if source_frame != msg.header.frame_id:
            rospy.logerr(
                "Source frame set to {}, but selected frame is {}".format(source_frame, msg.header.frame_id)
            )
            rospy.logerr("Please select the %s frame", source_frame)
            return

        try:
            rospy.loginfo("Transforming pose from {} to {}".format(source_frame, map_frame))
            trans = self.tf_buffer.lookup_transform(
                map_frame,
                source_frame,
                look_up_time,
                rospy.Duration(self.config.timeout),
            )

        except tf2.LookupException as ex:
            rospy.logwarn_throttle(3, str(look_up_time.to_sec()))
            rospy.logwarn_throttle(3, ex)
            rospy.logwarn_throttle(
                3, "Could not get transformation from {} to {}".format(source_frame, map_frame)
            )
            return
        except tf2.ExtrapolationException as ex:
            rospy.logwarn_throttle(3, str(look_up_time.to_sec()))
            rospy.logwarn_throttle(3, ex)
            return

        pose_out = do_transform_pose(msg, trans)
        self.pub_new_goal.publish(pose_out)

    def home_callback(self, home_msg):
        """Callback function that stores the assigned goal pose to the class
        :param home_msg: Curren goal
        :return: None
        """
        rospy.loginfo("Saved home position")
        self.home_pose = PoseStamped()
        self.home_pose.header = home_msg.header
        self.home_pose.pose = home_msg.pose.pose

    def home_server(self, _):
        """Service server
        :param _: req not used
        """
        if self.home_pose is not None:
            rospy.loginfo("Initiated come home")
            self.home_pose.header.stamp = rospy.Time()
            self.home = True
            self.goal_callback(self.home_pose)
            return TriggerResponse(success=True, message="Initiated come home")
        rospy.logerr("No home positon defined")
        return TriggerResponse(success=False, message="No home position defined")

    def shutdown(self):
        """Used to shutdown service server and unregister pub/sub"""
        self.sub_goal.unregister()
        self.pub_new_goal.unregister()
        self.srv_home.shutdown()


if __name__ == "__main__":
    home_pose_tf = None
    try:
        rospy.init_node("comehome_pose_tf")
        home_pose_tf = ComehomePoseTF2()

        rospy.spin()
    except rospy.ROSInterruptException:
        home_pose_tf.shutdown()
        # pass
