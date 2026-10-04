cmake_minimum_required(VERSION 3.20)
project(mission_planner)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(drone_interfaces REQUIRED)

add_executable(planner_node
  src/planner_node.cpp
)
target_link_libraries(planner_node PRIVATE
  rclcpp::rclcpp
  nav_msgs::nav_msgs
  geometry_msgs::geometry_msgs
  sensor_msgs::sensor_msgs
  std_msgs::std_msgs
  ${drone_interfaces_TARGETS}
)

install(TARGETS
  planner_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(
  rclcpp
  nav_msgs
  geometry_msgs
  sensor_msgs
  drone_interfaces
)

ament_package()
