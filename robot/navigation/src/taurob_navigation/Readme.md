# Taurob Navigation
Customized move base flex launch files and configuration for the taurob tracker.<br />
SMACH behavior for the automatic navigation.

## taurob_smach.py
Currently working:
- [x] Sending goal via rviz
- [x] Switching between teb & dwa planer
- [x] Switching between recovery behaviors
- [x] Move back recovery
- [x] Clear costmap recovery
- [ ] Rotation recovery


## taurob_smach_replan.py
Currently working:
- [x] Sending goal via rviz
- [x] Sending new goal via rviz and replaning path to the new goal
- [x] Switching between teb & dwa planer
- [x] Switching between recovery behaviors
- [x] Move back recovery
- [x] Clear costmap recovery
- [ ] Rotation recovery

## Subscribed Topics
* **` /move_base_simple/goal`** ([geometry_msgs::PoseStamped])

## Published actions
* **`mbf_actions for automatic drive`**