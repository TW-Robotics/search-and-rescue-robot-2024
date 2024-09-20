# Comehome Pose TF2
Manages the given home position and waits for the service call to start driving home.  
Handels the navigation goals given by the operator via rviz. Should the given position not oiginate from the map frame (current frame in rviz) it will transforms the given position into the map frame. MBF needs the position in the map_frame otherwise an error will be returned.

## Subscribed Topics
* **`/rviz/goal`** ([geometry_msgs::PoseStamped])
* **`/home_goal`** ([geometry_msgs::PoseWithCovarianceStamped])

The subscribed topics are published via RVIZ where the default topics of the buttons are remapped (using the `Tool Properties`):
* **`/rviz/goal`**: 2D Nav Goal buttion
* **`/home_goal`**: 2D Pose Estimate buttion

## Published actions
* **`/move_base_simple/goal`** ([geometry_msgs::PoseStamped])

## Service Server
* **`/Comehome_GoalPoseTF/drive_home`** ([std_srvs::Trigger])

## Configuration
Configurations can be set via ROS Parameter Server (online) or via dynamic reconfigure