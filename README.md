IRIS AUV NETTAG Acousitc and Vision control stack for use in Stonefish

To use this control pipeline first follow all instructions on https://github.com/lmbalves/iris_stonefish/blob/master/README.md

After you have averything runnig go ahead and clone this repo to your catkin workspace and build

Files/Nodes

 - setpoints_mkII.cpp and setpoints_pub_iris2.cpp - two different approaches use the keyboard to control IRIS
 - acoustic_pilot.cpp - given the tag localization (heading estimation node TODO) will publish commands for speed and yaw through std_msgs::Twist following both a circular and a descendig path towards the tag and circle round the tag to a defined radius
 - setpoints_visual_navigation.cpp - using trained model will detect and approach fishing apparell lost in the sea
 - thrust_setpoints_pub.cpp - subscribes std_msgs::Twist messages and publishes setpoints to the thrusters
 - plot_path.py - use this plotter to graph realtime the path followed by the IRIS

TODO

 - A heading estimation node that using the know waypoints triangulates to understand the localization of the tag

 - A captain node that will cycle trough running modes, first it will circle util it finds the x,y realtive location the it will descend to understand z realtive location, heading estimation will then be able to publish a location, then it kicks the coarse approach and finaly it will use the visual navigation to detect and make the final approach to the net.