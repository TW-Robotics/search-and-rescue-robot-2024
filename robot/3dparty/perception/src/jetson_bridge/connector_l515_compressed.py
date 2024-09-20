import base64
import logging
import time

import roslibpy
import rospy

import numpy as np

from sensor_msgs.msg import Image, CompressedImage

client = roslibpy.Ros(host='10.0.0.105', port=9090)
rospy.init_node("jetson_connector_l515_color")

class debugger:
    def __init__(self):
        self.latestImage = None
        self.pub = rospy.Publisher("/L515/color/image_raw/compressed", CompressedImage, queue_size=1)
        self.subscriber = roslibpy.Topic(client, '/L515/color/image_raw/compressed', 'sensor_msgs/CompressedImage')
        self.subscriber.subscribe(self.receive_image)
    #
    def receive_image(self, msg):
        self.latestImage = msg
        self.updateMessage()
    #
    def updateMessage(self):
        rosmsg = CompressedImage()
        rosmsg.header.frame_id = self.latestImage["header"]["frame_id"]
        rosmsg.format = "jpeg"
        base64_bytes = self.latestImage['data'].encode('ascii')
        rosmsg.data = base64.b64decode(base64_bytes)
        self.pub.publish(rosmsg)


test = debugger()
client.run()

rospy.spin()