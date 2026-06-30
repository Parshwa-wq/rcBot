cmake_minimum_required(VERSION 3.8...3.25)
project(ocpkg)

# Ensure we use the system Python that has ROS 2 dependencies
set(Python3_EXECUTABLE /usr/bin/python3)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate custom messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/EncoderTicks.msg"
  "msg/MechanismState.msg"
  "msg/MotorCmds.msg"
  "msg/WheelDistances.msg"
  DEPENDENCIES std_msgs
)

# Install Python scripts
install(PROGRAMS
  scripts/ps5_teleop_node.py
  scripts/mechanism_node.py
  scripts/yaw_manager_node.py
  scripts/pid_controller_node.py
  scripts/signal_display_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
