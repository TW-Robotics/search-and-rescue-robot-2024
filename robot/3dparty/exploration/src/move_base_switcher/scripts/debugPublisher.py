#! /usr/bin/env python
import rospy
from std_msgs.msg import Bool

if __name__ == '__main__':
    rospy.init_node('manual_move_base_action_switcher')
    pub = rospy.Publisher('/explorer/enable', Bool, queue_size=1)
    while not rospy.is_shutdown():
        tmpIn = input()
        try:
            tmpIn = int(tmpIn)
        except ValueError:
            break
        else:
            pubBool = Bool(tmpIn == 1)
            pub.publish(pubBool)