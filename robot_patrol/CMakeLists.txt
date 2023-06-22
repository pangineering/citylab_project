cmake_minimum_required(VERSION 3.5)
project(robot_patrol)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(robot_interfaces REQUIRED)
find_package(rclcpp_action REQUIRED)
add_executable(robot_patrol_node src/patrol.cpp)
ament_target_dependencies(robot_patrol_node rclcpp sensor_msgs)

add_executable(robot_patrol_node_2 src/patrol_v2.cpp)
ament_target_dependencies(robot_patrol_node_2 rclcpp sensor_msgs std_msgs geometry_msgs robot_interfaces)

add_executable(direction_service_node src/direction_service.cpp)
ament_target_dependencies(direction_service_node rclcpp sensor_msgs std_msgs geometry_msgs robot_interfaces)

add_executable(test_service_node src/test_service.cpp)
ament_target_dependencies(test_service_node rclcpp sensor_msgs std_msgs geometry_msgs robot_interfaces)

add_executable(go_to_point_action_node src/go_to_point_action.cpp)
ament_target_dependencies(go_to_point_action_node rclcpp sensor_msgs std_msgs geometry_msgs nav_msgs robot_interfaces rclcpp_action)

install(TARGETS
   robot_patrol_node
   robot_patrol_node_2
   direction_service_node
   go_to_point_action_node
   test_service_node
   DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_export_dependencies(ament_cmake)
ament_export_dependencies(rclcpp)
ament_export_dependencies(sensor_msgs)
ament_export_dependencies(std_msgs)
ament_export_dependencies(nav_msgs)
ament_export_dependencies(geometry_msgs)
ament_export_dependencies(robot_interfaces)

ament_package()
