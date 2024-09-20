# 3D Party Perception


* **laser_merge**: Combines filtered laserscans into one.
* **lidar_footprint_filter**: Filters 3D cloud to exclude points representing the robot.
* **pointcloud2laserscan**: Converts 3D cloud from Velodyne into a laser scan.

* **realsense**: Realsense camera driver (depreciated in favor of jetson_bridge).
* **jetson_bridge**: Uses ROS web tools to connect to Jetson running Realsense cameras.