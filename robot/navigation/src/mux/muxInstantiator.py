#! /usr/bin/env python

import json
import sys
import subprocess
import signal
import os
import rospy

def rosMasterRunning():
    """Checks if master is up
    """
    return "/rosout" in str(
        subprocess.Popen(
            ["rostopic list"],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            shell=True
        ).communicate()
    )

if __name__ == '__main__':
    # Init dummy node
    rospy.init_node("mux_dummy")

    with open('/app/muxConfig.json') as json_file:
        muxConfig = json.load(json_file)

    p = []

    for entry in muxConfig["mux"]:
        # Log started configs
        rospy.loginfo("Starting Mux Node using:\n{}".format(
            str([sys.executable, '/app/genericMux.py', str(entry)])
        ))

        # Create mux node based on the provided config
        p.append(
            subprocess.Popen([sys.executable, '/app/genericMux.py', str(entry)], preexec_fn=os.setpgrp)
        )
    
    # Keep this node running to keep the container alive
    while rosMasterRunning():
        rospy.rostime.wallsleep(4)

    # Kill nodes when master is down
    for entry in p:
        os.killpg(entry.pid, signal.SIGTERM)
