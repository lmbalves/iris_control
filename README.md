IRIS AUV NETTAG Acousitc and Vision control stack for use in Stonefish

To use this control pipeline first follow all instructions on https://github.com/lmbalves/iris_stonefish/blob/master/README.md

After you have everything running, clone this repo into the same ROS 2 (Lyrical) workspace and build with `colcon build`.

Run nodes with e.g. `ros2 run iris_control thrust_setpoints_pub` and `ros2 run iris_control acoustic_pilot`, or `ros2 launch iris_control iris_control.launch.py`.

Files/Nodes

 - tag_range_estimator - estimates the vehicle->tag vector in level NED from USBL ranges only (bearing is ignored), using IMU + DVL dead reckoning. Linear Least Squares init, then Huber-weighted damped Gauss Newton over a sliding window, with a converged latch (hysteresis at 2x threshold). While not converged it publishes a virtual tag at the start point so the pilot loiters in a circle while pings accumulate. The "tag below sensor" prior only applies while the ping window is depth-flat, otherwise the vehicle kept descending with the tag estimate. Optional tag_depth parameter removes the ambiguity when the tag depth is known. 
 - estimated_acoustic_pilot - same guidance and control law as acoustic_pilot, but consumes /iris/tag_rel  instead of the USBL, no internal dead reckoning, and holds still if tag_rel goes stale (tag_timeout).
 - setpoints_mkII.cpp and setpoints_pub_iris2.cpp - two different approaches use the keyboard to control IRIS
 - acoustic_pilot.cpp - given the tag localization (heading estimation node TODO) will publish commands for speed and yaw through std_msgs::Twist following both a circular and a descendig path towards the tag and circle round the tag to a defined radius
 - setpoints_visual_navigation.cpp - using trained model will detect and approach fishing apparell lost in the sea
 - thrust_setpoints_pub.cpp - subscribes std_msgs::Twist messages and publishes setpoints to the thrusters
 - plot_path.py - use this plotter to graph realtime the path followed by the IRIS

TODO

 - A captain node that will cycle trough running modes, first it will circle util it finds the x,y realtive location the it will descend to understand z realtive location, heading estimation will then be able to publish a location, then it kicks the coarse approach and finaly it will use the visual navigation to detect and make the final approach to the net.
