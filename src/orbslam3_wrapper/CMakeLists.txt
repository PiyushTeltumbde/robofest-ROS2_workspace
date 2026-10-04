cmake_minimum_required(VERSION 3.20)
project(orbslam3_wrapper)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_definitions(-DCOMPILEDWITHC11 -DHAVE_EPOXY)

# ------------------------------------------------------------------------------
# Dependencies
# ------------------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

find_package(OpenCV REQUIRED)
find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(Boost REQUIRED COMPONENTS serialization)

# ------------------------------------------------------------------------------
# ORB-SLAM3 Real Path
# ------------------------------------------------------------------------------
set(ORB_SLAM3_DIR "/home/harsh-mahamuni/ORB_SLAM3" CACHE PATH "Path to ORB_SLAM3 root")

message(STATUS "ORB-SLAM3 root directory: ${ORB_SLAM3_DIR}")

if(NOT EXISTS "${ORB_SLAM3_DIR}/lib/libORB_SLAM3.so")
  message(FATAL_ERROR "libORB_SLAM3.so not found at ${ORB_SLAM3_DIR}/lib/libORB_SLAM3.so")
endif()

include_directories(
  include
  ${ORB_SLAM3_DIR}
  ${ORB_SLAM3_DIR}/include
  ${ORB_SLAM3_DIR}/include/CameraModels
  ${ORB_SLAM3_DIR}/Thirdparty/Sophus
  ${ORB_SLAM3_DIR}/Thirdparty/DBoW2
  ${ORB_SLAM3_DIR}/Thirdparty/g2o
  ${EIGEN3_INCLUDE_DIR}
  ${Pangolin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

set(ORB_SLAM3_EXTERNAL_LIBS
  ${ORB_SLAM3_DIR}/lib/libORB_SLAM3.so
  ${ORB_SLAM3_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
  ${ORB_SLAM3_DIR}/Thirdparty/g2o/lib/libg2o.so
  ${Pangolin_LIBRARIES}
  ${OpenCV_LIBS}
  ${Boost_LIBRARIES}
  crypto
)

set(ORB_SLAM3_RPATHS
  "${ORB_SLAM3_DIR}/lib;${ORB_SLAM3_DIR}/Thirdparty/DBoW2/lib;${ORB_SLAM3_DIR}/Thirdparty/g2o/lib;/usr/local/lib"
)

# ------------------------------------------------------------------------------
# Executable
# ------------------------------------------------------------------------------
add_executable(orbslam3_node
  src/orbslam3_node.cpp
)

target_include_directories(orbslam3_node PRIVATE
  include
  ${ORB_SLAM3_DIR}
  ${ORB_SLAM3_DIR}/include
  ${ORB_SLAM3_DIR}/include/CameraModels
  ${ORB_SLAM3_DIR}/Thirdparty/Sophus
  ${ORB_SLAM3_DIR}/Thirdparty/DBoW2
  ${ORB_SLAM3_DIR}/Thirdparty/g2o
  ${EIGEN3_INCLUDE_DIR}
  ${Pangolin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(orbslam3_node PRIVATE
  rclcpp::rclcpp
  sensor_msgs::sensor_msgs
  geometry_msgs::geometry_msgs
  nav_msgs::nav_msgs
  std_msgs::std_msgs
  cv_bridge::cv_bridge
  ${ORB_SLAM3_EXTERNAL_LIBS}
)

set_target_properties(orbslam3_node PROPERTIES
  BUILD_RPATH "${ORB_SLAM3_RPATHS}"
  INSTALL_RPATH "${ORB_SLAM3_RPATHS}"
)

# ------------------------------------------------------------------------------
# Install & Export
# ------------------------------------------------------------------------------
install(TARGETS
  orbslam3_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  std_msgs
  cv_bridge
)

ament_package()
