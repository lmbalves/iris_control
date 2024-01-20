cmake_minimum_required(VERSION 3.0.2)
project(iris_control)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  # ROS Dependencies
  cv_bridge
  image_transport

  rosbridge_server
  tf
  tf2
  tf2_ros
  topic_tools
  rviz
  diagnostic_aggregator
  cmake_modules
  roscpp
  rosparam
  rospy
  roslib
  std_msgs
  std_srvs
  sensor_msgs
  geometry_msgs
  visualization_msgs
  diagnostic_msgs
  actionlib
  stonefish_ros
  message_generation
)


#ament_python_install_package(${PROJECT_NAME})

# System Dependencies
find_package(Boost REQUIRED)
#find_package(COLA2_LIB REQUIRED)



# Declare things to be passed to dependent projects
catkin_package(
  INCLUDE_DIRS
  LIBRARIES 
  # TODO: go and check what is in the headers
  CATKIN_DEPENDS roscpp
                 roslib
                 std_msgs
                 sensor_msgs
                 actionlib
                 stonefish_ros
                #  cola2_lib_ros
                #  cola2_msgs
  DEPENDS Boost

)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

#install(PROGRAMS
#  /src/path_motor_control2.py
#  DESTINATION lib/${iris_control}
#)
catkin_python_setup()

catkin_install_python(PROGRAMS  
  src/path_motor_control2.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
catkin_install_python(PROGRAMS  
  src/control_vision.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS  
  src/control_rotors.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS  
  src/plot_path.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#adicionei estes 3 para testar um codigo diferente
add_executable(setpoints_pub_node2 src/setpoints_pub_iris2.cpp)  
add_dependencies(setpoints_pub_node2 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(setpoints_pub_node2 ${catkin_LIBRARIES})

add_executable(setpoints_pub_node src/setpoints_pub_iris.cpp)
add_dependencies(setpoints_pub_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(setpoints_pub_node ${catkin_LIBRARIES}) 

add_executable(setpoints_mkII src/setpoints_mkII.cpp)
add_dependencies(setpoints_mkII ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(setpoints_mkII ${catkin_LIBRARIES})

add_executable(acoustic_pilot src/acoustic_pilot.cpp)
add_dependencies(acoustic_pilot ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(acoustic_pilot ${catkin_LIBRARIES})

add_executable(thrust_setpoints_pub src/thrust_setpoints_pub.cpp)
add_dependencies(thrust_setpoints_pub ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(thrust_setpoints_pub ${catkin_LIBRARIES})

add_executable(setpoints_visual_navigation src/setpoints_visual_navigation.cpp)
add_dependencies(setpoints_visual_navigation ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(setpoints_visual_navigation ${catkin_LIBRARIES})

add_executable(pilot src/pilot.cpp)
add_dependencies(pilot ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pilot ${catkin_LIBRARIES})
