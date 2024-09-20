#!/bin/bash
echo "User: $(whoami)"
/opt/ros/noetic/env.sh /bin/bash -c "until rostopic list > /dev/null 2>&1; do sleep 1; done; sleep 5; until ! rostopic list > /dev/null 2>&1; do sleep 1; done; sudo service compose@"$1" restart" & 
PID="$! "
echo -e "\e[32mROS Check running at $PID with user: $(whoami)\e[0m"