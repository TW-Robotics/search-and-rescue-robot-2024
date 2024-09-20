# Exploration

Builds [explore-lite](https://github.com/hrnr/m-explore) exploration packages locally and starts them. Exploration nodes are started in the ```/explorer``` namespace to intercept communication between exploration package and move_base. A separate *fake_move_base_server* node forwards movement actions from the explorer to move_base.

By default, no commands are forwarded. Exploration can be started or stopped by sending a simple boolean containing *true* or *false* to the ```/explorer/enable``` topic.

## Building

By default, ```ROS_MASTER_URI```, ```ROS_HOSTNAME``` and ```ROS_IP``` are set on buildtime for the tracker robot. If you want to test the docker image locally on a single PC, you can build it with the build argument BUILD_RELEASE set to an empty string: ```docker build --rm -t taurob/explorer -f Dockerfile.explorer . --build-arg BUILD_RELEASE=""```

## Parameters

Explorer parameters can be set [here](./src/move_base_switcher/launch/capsulated_explorer.launch). The only explorer parameter that is not at its default value is *min_frontier_size*. *min_frontier_size* has been increased from 0.75m to 1.5m to match the tracker's width.