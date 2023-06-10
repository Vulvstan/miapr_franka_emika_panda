cmake_minimum_required(VERSION 3.8)
project(franka_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
include_directories(include)
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(franka_control_services REQUIRED)
find_package(std_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(control_msgs REQUIRED)
find_package(moveit_common REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(franka_control_services REQUIRED)

include_directories(
  include
  /opt/ros/rolling/include
  /opt/ros/rolling/include/urdf
  /opt/ros/rolling/include/moveit_msgs
  /opt/ros/rolling/include/geometry_msgs
  /opt/ros/rolling/include/trajectory_msgs
  /opt/ros/rolling/include/sensor_msgs
  /opt/ros/rolling/include/visualization_msgs
  /opt/ros/rolling/include/object_recognition_msgs
  /opt/ros/rolling/include/shape_msgs
  /opt/ros/rolling/include/octomap_msgs
  /opt/ros/rolling/include/unique_identifier_msgs
  /opt/ros/rolling/include/action_msgs
  /opt/ros/rolling/include/rclcpp_action
  /opt/ros/rolling/include/rcl_action
  /opt/ros/rolling/include/tf2_msgs
  /opt/ros/rolling/include/pluginlib
  /opt/ros/rolling/include/class_loader
  /opt/ros/rolling/include/graph_msgs
  ${colcon_INCLUDE_DIRS}
)

add_executable(trajectory_server src/trajectory_server.cpp)
ament_target_dependencies(trajectory_server rclcpp moveit_ros_planning_interface franka_control_services) 


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Install targets
install(TARGETS
  trajectory_server
  DESTINATION lib/${PROJECT_NAME})

ament_package()
