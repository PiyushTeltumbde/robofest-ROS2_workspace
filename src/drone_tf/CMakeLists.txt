cmake_minimum_required(VERSION 3.20)
project(drone_tf)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

add_executable(tf_broadcaster
  src/tf_broadcaster.cpp
)
target_link_libraries(tf_broadcaster PRIVATE
  rclcpp::rclcpp
  tf2::tf2
  tf2_ros::tf2_ros
  geometry_msgs::geometry_msgs
  nav_msgs::nav_msgs
)

install(TARGETS
  tf_broadcaster
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(
  rclcpp
  tf2
  tf2_ros
  geometry_msgs
  nav_msgs
)

ament_package()
