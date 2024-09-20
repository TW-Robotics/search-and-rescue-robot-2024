# Installation Guide

0. Make sure that host PC is running a supported Kernel version. For Ubuntu 22.04, only kernel 5.15 is supported. You might have to use [mainline kernel utility](https://github.com/bkw777/mainline) to install an older kernel. Run `bash installMainline.sh` to install the utility and select kernel 5.15.
1. Connect cameras and run `bash installRealseneDKMSOnHost.sh` to install required kernel modules on the host.
2. Run `realsense-viewer` to verify the cameras work, data rates are fine and update camera firmware (firmware makes or breaks compatibility).

# Usage

1. Build and run the container using `docker-compose up --build` within this directory. Alternatively, you can run `docker run -it --privileged -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix taurob/perception:realsense` for debugging with a forwarded GUI.
2. Set the **BUILD_RELEASE** argument to **""** in order to do local debugging. This prevents *ROS_MASTER_URI* and *ROS_HOSTNAME* from being set for deployment on the real robot.