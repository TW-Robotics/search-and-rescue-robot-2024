#! /usr/bin/env python

import sys
import signal
import ast # ast instead of json, see: https://stackoverflow.com/questions/4162642/single-vs-double-quotes-in-json
import rospy
# Utilised message types must be imported here
from geometry_msgs.msg import PoseStamped, Twist

class UtilSub:
    """Utility subscriber to handle subbing to each mux input. Stores latest message and timestamp
    """
    def __init__(self, topic, dtype):
        """Class constructor
        """
        self.latestMsg = None
        self.latestMsgStamp = None
        self.updated = False
        self.sub = rospy.Subscriber(topic, dtype, self.callback)

    def callback(self, data):
        """Subscriber callback
        """
        self.latestMsg = data
        self.latestMsgStamp = rospy.get_time()
        self.updated = True

    def getLatestMsg(self):
        """Getter for latest message and timestamp
        """
        self.updated = False
        return self.latestMsg, self.latestMsgStamp
    
    def isUpdated(self):
        """Getter for bool indicating that a message has been received
        """
        return self.updated

class Mux:
    """Mux instance that is configured from config dict (see config.json in ros package)
    """
    def __init__(self, configDict):
        # Store config and utility variables
        dtype = eval(configDict["dType"])                               # Published/Subscribed message type
        self.cyclesSinceInputChange = configDict["cooldownCycles"]      # Counts updates since input change to honor set cooldown
        self.latestInput = 0                                            # Tracks latest input priority
        self.config = configDict

        # Instantiate subscribers and publisher
        self.subscribers = [ UtilSub(topic, dtype) for topic in configDict["topics"] ]  # Subscribers in descending priority
        self.publisher = rospy.Publisher(configDict["outTopic"], dtype, queue_size=0)

    def incrementCycles(self):
        """Increments cycle since update count
        """
        if self.cyclesSinceInputChange < self.config["cooldownCycles"]:
            self.cyclesSinceInputChange += 1

    def update(self):
        """Polls subscriber updates
        """
        # Check for incoming poses
        updateList =  [ s.isUpdated() for s in self.subscribers ]
        updateIndices = [ i for i, c in enumerate(updateList) if c ]

        #rospy.loginfo("Mux Update Called")
        #rospy.loginfo(updateList)

        # If poses have been updated, check if we need to switch input, otherwise abort update
        if len(updateIndices) <= 0:
            return

        # Only hand off control to input of same or higher priority of after cooldown period
        if updateIndices[0] <= self.latestInput or self.cyclesSinceInputChange >= self.config["cooldownCycles"]:
            # Store new latest input and reset cooldown
            self.latestInput = updateIndices[0]
            self.cyclesSinceInputChange = 0
            # Finally, publish message
            msg, _ = self.subscribers[updateIndices[0]].getLatestMsg()
            self.publisher.publish(msg)
        else:
            # Increment cooldown counter
            self.incrementCycles()

def signal_handler(signal, frame):
    print('\n')
    sys.exit(0)

def main():
    configDict = ast.literal_eval(sys.argv[1])
    # Load config and init node and mux
    rospy.init_node(configDict["name"])
    rate = rospy.Rate(1/configDict["dt"])
    mux = Mux(configDict)

    # Register sigint
    signal.signal(signal.SIGTERM, signal_handler)

    # Poll updates in regular intervals and publish accordingly
    while not rospy.is_shutdown():
        mux.update()
        rate.sleep()

if __name__ == '__main__':
    main()