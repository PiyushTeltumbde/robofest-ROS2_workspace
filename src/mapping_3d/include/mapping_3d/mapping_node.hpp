#ifndef MAPPING_3D__MAPPING_NODE_HPP_
#define MAPPING_3D__MAPPING_NODE_HPP_

#include <cmath>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <tf2/LinearMath/Transform.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>

namespace mapping_3d
{

struct Point3D
{
  float x{0.0f};
  float y{0.0f};
  float z{0.0f};
  uint64_t seq{0};
};

struct VoxelCoord
{
  int64_t x{0};
  int64_t y{0};
  int64_t z{0};

  bool operator==(const VoxelCoord & other) const
  {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct VoxelCoordHash
{
  std::size_t operator()(const VoxelCoord & k) const
  {
    std::size_t h1 = std::hash<int64_t>{}(k.x);
    std::size_t h2 = std::hash<int64_t>{}(k.y);
    std::size_t h3 = std::hash<int64_t>{}(k.z);
    return h1 ^ (h2 << 1) ^ (h3 << 2);
  }
};

class MappingNode : public rclcpp::Node
{
public:
  explicit MappingNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  virtual ~MappingNode() = default;

private:
  void declare_and_get_parameters();
  void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void depth_callback(const sensor_msgs::msg::Image::SharedPtr msg);
  void publish_timer_callback();

  bool find_closest_pose(const rclcpp::Time & depth_stamp, nav_msgs::msg::Odometry & closest_odom);
  bool compute_camera_to_map_transform(
    const nav_msgs::msg::Odometry & odom,
    const std::string & depth_camera_frame,
    tf2::Transform & T_map_cam);
  void add_points_to_global_map(const std::vector<Point3D> & points);
  sensor_msgs::msg::PointCloud2 create_cloud_msg(
    const std::vector<Point3D> & points,
    const rclcpp::Time & stamp,
    const std::string & frame_id);

  // Parameters
  std::string pose_topic_{"/drone/odometry"};
  std::string depth_topic_{"/camera/depth/image_raw"};
  std::string camera_info_topic_{"/camera/camera_info"};
  std::string map_frame_{"map"};
  std::string camera_frame_{"camera_optical_frame"};
  std::string base_frame_{"base_link"};
  double min_depth_m_{0.2};
  double max_depth_m_{8.0};
  int pixel_stride_{4};
  double voxel_size_m_{0.05};
  int max_points_{200000};
  int max_map_points_{1000000};
  double max_pose_age_sec_{0.2};
  double publish_rate_hz_{5.0};

  // State synchronization and protection
  std::mutex cam_info_mutex_;
  bool has_camera_info_{false};
  double fx_{0.0};
  double fy_{0.0};
  double cx_{0.0};
  double cy_{0.0};

  std::mutex odom_mutex_;
  bool has_odom_{false};
  rclcpp::Time last_odom_stamp_;
  std::deque<nav_msgs::msg::Odometry> pose_history_;

  std::mutex depth_state_mutex_;
  bool has_depth_{false};
  rclcpp::Time last_depth_stamp_;

  std::mutex map_mutex_;
  std::unordered_map<VoxelCoord, Point3D, VoxelCoordHash> voxel_map_;
  uint64_t point_sequence_{0};

  // TF2
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // ROS Interfaces
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr current_cloud_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr status_pub_;

  rclcpp::TimerBase::SharedPtr publish_timer_;
};

}  // namespace mapping_3d

#endif  // MAPPING_3D__MAPPING_NODE_HPP_
