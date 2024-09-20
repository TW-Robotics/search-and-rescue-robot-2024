#! /usr/bin/env python

import roslib
roslib.load_manifest("move_base_switcher")
import rospy
import actionlib

from move_base_msgs.msg import MoveBaseAction
from actionlib_msgs.msg import GoalStatus

from geometry_msgs.msg import PoseStamped

from dynamic_reconfigure.server import Server
from move_base_switcher.cfg import enableExplorerConfig

import subprocess

"""Provides a fake move_base action server to explore_lite, stores the goal and publishes it as a goalpose
"""

class FakeMoveBaseServer:
    """ SimpleActionServer that mimicks the behaviour of the move_base action server
    """
    def __init__(self, ns="explorer", goaldt=8):
        """Class Constructor
        """
        self.goalbuffer = None
        self.forwardCommands = False
        self.goaldt = goaldt
        # cmd_vel publisher
        self.posePub = rospy.Publisher("{}/goalpose".format(ns), PoseStamped, queue_size=0)
        self.posePubVis = rospy.Publisher("{}/goalpose/visualisation".format(ns), PoseStamped, queue_size=0)
        # Action server to connect to explorer
        self.server = actionlib.SimpleActionServer("{}/move_base".format(ns), MoveBaseAction, self.executeAction, False)
        self.server.start()
    #
    def executeAction(self, goal):
        """Action Callback. Stores and publishes current goalpose
        """
        rospy.loginfo("Fake move_base: Received action")
        # Store goal in case we get interrupted. Type=Posestamped
        self.goalbuffer = goal.target_pose
        # We do not set the action to succeeded here, because we want to wait until the robot has discovered that frontier
        if self.forwardCommands is True:
            # If we are in forwarding mode, then send commands
            self.posePub.publish(self.goalbuffer)
        else:
            # Otherwise, do nothing
            pass
        # Always send pose to the visualisation channel
        self.posePubVis.publish(self.goalbuffer)
        # Set goal to succeeded anyways, since explore-lite does not have error handling
        # Wait before sending update
        rospy.rostime.wallsleep(self.goaldt)   
        # Set succeeded
        self.server.set_succeeded()
        return True
    #
    def enable(self, switch):
        """Updates to forward/not forward commands based on "switch".
           Interrupted commands are buffered and aborted
        """
        if switch is False and self.forwardCommands is True:
            # Deactivate forwarding
            self.forwardCommands = False
            # Cancel all goals by publishing goal of current pose
            subprocess.run(["rostopic pub --once preempted/goalpose geometry_msgs/PoseStamped '{header: {stamp: now, frame_id: 'base_link'}, pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}'"], shell=True, capture_output=True, text=True)
            # Debug message
            rospy.loginfo("Fake move_base: Stopped forwarding fommands")
        elif switch is True and self.forwardCommands is False:
            # Activate forwarding
            self.forwardCommands = True
            # Resume buffered goal
            self.posePub.publish(self.goalbuffer)
            # Debug message
            rospy.loginfo("Fake move_base: Started forwarding commands")

    
class configureWrapper:
    def __init__(self):
        self.moveBaseSwitcher = FakeMoveBaseServer()
        self.server = Server(enableExplorerConfig, self.callback)

    def callback(self, config, level):
        rospy.loginfo("Dynamic reconfigure switched with this configuration: \n{}".format(config))
        self.moveBaseSwitcher.enable(config["enable_explorer"])
        return config


if __name__ == '__main__':
    rospy.init_node('fake_move_base_server')
    server = configureWrapper()
    rospy.spin()
