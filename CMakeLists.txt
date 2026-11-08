cmake_minimum_required(VERSION 3.8)
project(iris_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(stonefish_ros2 REQUIRED)

# Find Eigen3
find_package(Eigen3 REQUIRED)

include_directories(include)

# Acoustic pilot node
add_executable(acoustic_pilot
  src/acoustic_pilot.cpp
)
target_link_libraries(acoustic_pilot
  rclcpp::rclcpp
  ${std_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  tf2_ros::tf2_ros
  tf2_geometry_msgs::tf2_geometry_msgs
  ${sensor_msgs_TARGETS}
  ${visualization_msgs_TARGETS}
  tf2::tf2
  ${stonefish_ros2_TARGETS}
)
target_include_directories(acoustic_pilot PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR}
)

# Thrust setpoints publisher node
add_executable(thrust_setpoints_pub
  src/thrust_setpoints_pub.cpp
)
target_link_libraries(thrust_setpoints_pub
  rclcpp::rclcpp
  ${std_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
)
target_include_directories(thrust_setpoints_pub PUBLIC
  ${EIGEN3_INCLUDE_DIR}
)

install(TARGETS
  acoustic_pilot
  thrust_setpoints_pub
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Add Python installation
install(PROGRAMS
  scripts/plot_path.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()