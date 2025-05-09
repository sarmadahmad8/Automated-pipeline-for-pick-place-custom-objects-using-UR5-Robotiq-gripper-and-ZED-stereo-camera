cmake_minimum_required(VERSION 3.8)
project(detection_recievers)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(moveit REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(ur_robot_driver REQUIRED)

add_executable(detection_reciever src/move_group_reciever.cpp)
ament_target_dependencies(detection_reciever
  rclcpp
  moveit_core
  moveit_ros_planning_interface
  moveit_visual_tools
  moveit_msgs
  geometry_msgs
  std_msgs
  sensor_msgs
  control_msgs
  ur_robot_driver
)

add_executable(detection_reciever_realtime src/move_realtime.cpp)
ament_target_dependencies(detection_reciever_realtime
  rclcpp
  moveit_core
  moveit_ros_planning_interface
  moveit_visual_tools
  moveit_msgs
  geometry_msgs
  sensor_msgs
  control_msgs
  ur_robot_driver
)

install(TARGETS detection_reciever_realtime detection_reciever
  DESTINATION lib/${PROJECT_NAME}
)

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

ament_package()
