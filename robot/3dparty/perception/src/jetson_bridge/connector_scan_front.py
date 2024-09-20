import base64
import logging
import time

import roslibpy
import rospy

import numpy as np

from sensor_msgs.msg import LaserScan

client = roslibpy.Ros(host='10.0.0.105', port=9090)
rospy.init_node("jetson_connector_scan_front")

class debugger:
    def __init__(self):
        self.latestMessage = None
        self.pub = rospy.Publisher("/scan_front", LaserScan, queue_size=1)
        self.subscriber = roslibpy.Topic(client, '/scan_front', 'sensor_msgs/LaserScan')
        self.subscriber.subscribe(self.receive_image)
        print("Everything Set Up")
    #
    def receive_image(self, msg):
        self.latestMessage = msg
        self.updateMessage()
    #
    def updateMessage(self):
        rosmsg = LaserScan()
        #rosmsg.header.frame_id = self.latestMessage["header"]["frame_id"]
        rosmsg.header.frame_id = "L515_scan"
        rosmsg.angle_min = self.latestMessage["angle_min"]
        rosmsg.angle_max = self.latestMessage["angle_max"]
        rosmsg.angle_increment = self.latestMessage["angle_increment"]
        rosmsg.time_increment = self.latestMessage["time_increment"]
        rosmsg.scan_time = self.latestMessage["scan_time"]
        rosmsg.range_min = self.latestMessage["range_min"]
        rosmsg.range_max = self.latestMessage["range_max"]
        #
        rosmsg.ranges = np.array(test.latestMessage["ranges"]).astype(float).tolist()
        rosmsg.intensities = np.array(test.latestMessage["intensities"]).astype(float).tolist()
        #
        self.pub.publish(rosmsg)


test = debugger()
client.run()

rospy.spin()