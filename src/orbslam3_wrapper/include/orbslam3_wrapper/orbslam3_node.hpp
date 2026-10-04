#ifndef ORBSLAM3_WRAPPER__ORBSLAM3_NODE_HPP_
#define ORBSLAM3_WRAPPER__ORBSLAM3_NODE_HPP_

#include <memory>
#include <string>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>
#include <cv_bridge/cv_bridge.hpp>

namespace ORB_SLAM3
{
class System;
}

namespace orbslam3_wrapper
{

class ORBSLAM3Node : public rclcpp::Node
{
public:
  explicit ORBSLAM3Node(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~ORBSLAM3Node() override;

private:
  void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr & msg);
  void initialize_slam();

  // ROS parameters
  std::string vocabulary_file_;
  std::string settings_file_;
  std::string sensor_mode_;
  std::string camera_frame_;
  bool use_viewer_{false};

  // ORB-SLAM3 System instance and protection mutex
  std::unique_ptr<ORB_SLAM3::System> slam_system_;
  std::mutex mutex_track_;
  bool slam_initialized_{false};

  // Tracking odometry computation state
  bool has_prev_pose_{false};
  double prev_timestamp_{0.0};
  double prev_x_{0.0};
  double prev_y_{0.0};
  double prev_z_{0.0};

  // ROS publishers and subscriber
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr status_pub_;
};

}  // namespace orbslam3_wrapper

#endif  // ORBSLAM3_WRAPPER__ORBSLAM3_NODE_HPP_
