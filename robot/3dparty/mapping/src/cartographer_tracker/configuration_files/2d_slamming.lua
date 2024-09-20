-- Documentation: https://google-cartographer-ros.readthedocs.io/en/latest/configuration.html

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",  --The ROS frame ID to use for publishing submaps, the parent frame of poses, usually “map”
  
  tracking_frame = "imu", --The ROS frame ID of the frame that is tracked by the SLAM algorithm. If an IMU is used, it should
												--be at its position, although it might be rotated. A common choice is “imu_link”.
												
  published_frame = "base_link", --The ROS frame ID to use as the child frame for publishing poses. For example “odom” if an
												--"odom” frame is supplied by a different part of the system. In this case the pose of “odom” in the map_frame
												--will be published. Otherwise, setting it to “base_link” is likely appropriate
  publish_to_tf = true, -- Enable or disable providing of TF transforms.		
  odom_frame = "odom",			-- Only used if provide_odom_frame is true. The frame between published_frame and map_frame to be
											--used for publishing the (non-loop-closed) local SLAM result. Usually “odom”.
											
  provide_odom_frame = true, --If enabled, the local, non-loop-closed, continuous pose will be published as the odom_frame
											--in the map_frame.
											
  publish_frame_projected_to_2d = false,
  use_odometry = true,   --If enabled, subscribes to nav_msgs/Odometry on the topic “odom”. Odometry must be provided in
										--this case, and the information will be included in SLAM.
										
  use_nav_sat = false, 
  use_landmarks = false, 
										
  num_laser_scans = 1,  --Number of laser scan topics to subscribe to. Subscribes to sensor_msgs/LaserScan on the “scan”
									---topic for one laser scanner, or topics “scan_1”, “scan_2”, etc. for multiple laser scanners.
									
  num_multi_echo_laser_scans = 0,  --Number of multi-echo laser scan topics to subscribe to. Subscribes to sen-
													--sor_msgs/MultiEchoLaserScan on the “echoes” topic for one laser scanner, or topics “echoes_1”, “echoes_2”,
													--etc. for multiple laser scanners.
													
  num_subdivisions_per_laser_scan = 1,
													---Number of point clouds to split each received (multi-echo) laser scan into. Sub-
													--dividing a scan makes it possible to unwarp scans acquired while the scanners are moving. There is a corre-
													--sponding trajectory builder option to accumulate the subdivided scans into a point cloud that will be used for
													--scan matching.
  num_point_clouds = 0,  --Number of point cloud topics to subscribe to. Subscribes to sensor_msgs/PointCloud2 on the
										--“points2” topic for one rangefinder, or topics “points2_1”, “points2_2”, etc. for multiple rangefinders.
  lookup_transform_timeout_sec = 0.2, --Timeout in seconds to use for looking up transforms using tf2
  submap_publish_period_sec = 0.3,  --Interval in seconds at which to publish the submap poses, e.g. 0.3 seconds
  pose_publish_period_sec = 5e-3, --Interval in seconds at which to publish the submap poses, e.g. 0.3 seconds
  trajectory_publish_period_sec = 30e-3, --Interval in seconds at which to publish the trajectory markers, e.g. 30e-3 for 30
														--milliseconds.
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true
MAP_BUILDER.num_background_threads=4 --anzahl an cores
TRAJECTORY_BUILDER_2D.num_accumulated_range_data=10 --lesegeschwindigekit der scans
TRAJECTORY_BUILDER_2D.use_imu_data = true
TRAJECTORY_BUILDER_2D.max_range=10
TRAJECTORY_BUILDER_2D.min_range=0.3
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds=3
--TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.07
--TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
--TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window=0.
POSE_GRAPH.optimize_every_n_nodes=50



return options
