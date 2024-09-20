import rospy
from geometry_msgs.msg import PoseStamped, Twist

import time

class moveAbort:
    def __init__(self, cooldownTicks=0):
        self.timer = 0
        self.cooldownTicks = cooldownTicks
        self.msg = PoseStamped()
        self.msg.header.stamp = rospy.Time.now()
        self.msg.header.frame_id = "base_link"
        self.msg.pose.orientation.w = 1.0
        #
        self.goalpub = rospy.Publisher("preempted/goalpose", PoseStamped, queue_size=1)
        self.cmd_velsub = rospy.Subscriber("teleop/cmd_vel", Twist, self.callback)
    #
    def callback(self, cmd_vel):
        if self.timer <= 0:
            self.goalpub.publish(self.msg)
            self.timer = self.cooldownTicks
    #
    def updateTicks(self):
        if self.timer > 0:
            self.timer -= 1

def main():
    rospy.init_node("taurob_teleop_shutdown")
    
    freq = 10 # Hz
    cooldown = 5 # Seconds
    
    rate = rospy.Rate(freq)
    node = moveAbort(cooldownTicks=freq*cooldown)
 
    while not rospy.is_shutdown():
        node.updateTicks()
        print(node.timer)
        rate.sleep()

if __name__ == "__main__":
    main()