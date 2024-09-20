import subprocess
import rospy
import time

# Prerequisite: apt install fping
# Call Service: rosservice call /Comehome_GoalPoseTF/drive_home

def main():
    operatorIP = "10.0.0.102"
    #
    while not rospy.is_shutdown():
        result = None
        result = subprocess.run(["fping -c1 -t300 {}".format(operatorIP)], shell=True, capture_output=True, text=True)
        #
        if result.stdout == "":
            ## Operator is not reachable
            print("Operator not reachable. Going home.")
            #
            subprocess.run(["rostopic pub -1 /teleop/cmd_vel geometry_msgs/Twist -- '[0.0, 0.0, 0.0]' '[0.0, 0.0, 0.0]'"])
            # Call gohome service here
            subprocess.run(["rosservice call /Comehome_GoalPoseTF/drive_home"], shell=True, capture_output=True, text=True)
            # TODO: Check service success
        else:
            ## Operator is reachable
            print("Stopping going home.")
            # Abort move goal here

if __name__ == "__main__":
    main()