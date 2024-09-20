#! /usr/bin/env python

import json
import sys
import subprocess
import signal
import os
import rospy

if __name__ == '__main__':
    # Init dummy node
    rospy.init_node("jetson_connector_dummy")
    rospy.loginfo("Starting Jetson Rosbridge Connector")

    # Keep track of processes
    p = []
    
    # Start desired processes
    p.append(subprocess.Popen([sys.executable, '/app/connector_scan_front.py'], preexec_fn=os.setpgrp))
    p.append(subprocess.Popen([sys.executable, '/app/connector_d455_compressed.py'], preexec_fn=os.setpgrp))
    #p.append(subprocess.Popen([sys.executable, '/app/connector_l515_compressed.py'], preexec_fn=os.setpgrp))

    # Keep this node running to keep the container alive
    rospy.spin()

    # Kill nodes when master is down
    for entry in p:
        os.killpg(entry.pid, signal.SIGTERM)