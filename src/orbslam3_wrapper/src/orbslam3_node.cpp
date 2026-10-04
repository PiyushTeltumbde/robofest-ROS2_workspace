#include "orbslam3_wrapper/orbslam3_node.hpp"

#include <cmath>
#include <filesystem>
#include <iostream>

#include <sensor_msgs/image_encodings.hpp>
#include <System.h>
#include <Tracking.h>
#include <sophus/se3.hpp>

namespace orbslam3_wrapper
{

ORBSLAM3Node::ORBSLAM3Node(const rclcpp::NodeOptions & options)
: Node("orbslam3_node", options),
  slam_initialized_(false),
  has_prev_pose_(false),
  prev_timestamp_(0.0),
  prev_x_(0.0),
  prev_y_(0.0),
  prev_z_(0.0)
{
  // ---------------------------------------------------------------------------
  // Parameters
  // ---------------------------------------------------------------------------
  std::string default_voc = "/home/harsh-mahamuni/ORB_SLAM3/Vocabulary/ORBvoc.txt";
  std::string default_settings = "/home/harsh-mahamuni/ros2_ws/src/orbslam3_wrapper/config/orbslam3.yaml";

  declare_parameter<std::string>("vocabulary_file", default_voc);
  declare_parameter<std::string>("settings_file", default_settings);
  declare_parameter<std::string>("sensor_mode", "MONOCULAR");
  declare_parameter<std::string>("camera_frame", "camera_optical_frame");
  declare_parameter<bool>("use_viewer", false);

  vocabulary_file_ = get_parameter("vocabulary_file").as_string();
  settings_file_ = get_parameter("settings_file").as_string();
  sensor_mode_ = get_parameter("sensor_mode").as_string();
  camera_frame_ = get_parameter("camera_frame").as_string();
  use_viewer_ = get_parameter("use_viewer").as_bool();

  RCLCPP_INFO(get_logger(), "Initializing ORB-SLAM3 Node:");
  RCLCPP_INFO(get_logger(), "  vocabulary_file: %s", vocabulary_file_.c_str());
  RCLCPP_INFO(get_logger(), "  settings_file:   %s", settings_file_.c_str());
  RCLCPP_INFO(get_logger(), "  sensor_mode:     %s", sensor_mode_.c_str());
  RCLCPP_INFO(get_logger(), "  camera_frame:    %s", camera_frame_.c_str());
  RCLCPP_INFO(get_logger(), "  use_viewer:      %s", use_viewer_ ? "true" : "false");

  // ---------------------------------------------------------------------------
  // Output Topics
  // ---------------------------------------------------------------------------
  pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/orbslam/pose", 10);
  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/orbslam/odometry", 10);
  status_pub_ = create_publisher<std_msgs::msg::Bool>("/orbslam/tracking_status", 10);

  // ---------------------------------------------------------------------------
  // Initialize ORB-SLAM3 System
  // ---------------------------------------------------------------------------
  initialize_slam();

  // ---------------------------------------------------------------------------
  // Camera Stream Subscription
  // ---------------------------------------------------------------------------
  image_sub_ = create_subscription<sensor_msgs::msg::Image>(
    "/camera/image_raw", 10,
    std::bind(&ORBSLAM3Node::image_callback, this, std::placeholders::_1));

  RCLCPP_INFO(get_logger(), "Subscribed to /camera/image_raw. Waiting for image frames...");
}

ORBSLAM3Node::~ORBSLAM3Node()
{
  std::lock_guard<std::mutex> lock(mutex_track_);
  if (slam_system_) {
    RCLCPP_INFO(get_logger(), "Shutting down ORB-SLAM3 system...");
    slam_system_->Shutdown();
    slam_system_.reset();
    slam_initialized_ = false;
    RCLCPP_INFO(get_logger(), "ORB-SLAM3 system shutdown completed.");
  }
}

void ORBSLAM3Node::initialize_slam()
{
  if (!std::filesystem::exists(vocabulary_file_)) {
    RCLCPP_ERROR(get_logger(), "Vocabulary file does not exist: %s", vocabulary_file_.c_str());
    slam_initialized_ = false;
    return;
  }

  if (!std::filesystem::exists(settings_file_)) {
    RCLCPP_ERROR(get_logger(), "Settings file does not exist: %s", settings_file_.c_str());
    slam_initialized_ = false;
    return;
  }

  ORB_SLAM3::System::eSensor sensor_type = ORB_SLAM3::System::MONOCULAR;
  if (sensor_mode_ == "MONOCULAR") {
    sensor_type = ORB_SLAM3::System::MONOCULAR;
  } else if (sensor_mode_ == "STEREO") {
    sensor_type = ORB_SLAM3::System::STEREO;
  } else if (sensor_mode_ == "RGBD") {
    sensor_type = ORB_SLAM3::System::RGBD;
  } else if (sensor_mode_ == "IMU_MONOCULAR") {
    sensor_type = ORB_SLAM3::System::IMU_MONOCULAR;
  } else {
    RCLCPP_WARN(get_logger(), "Unknown sensor mode '%s', defaulting to MONOCULAR.", sensor_mode_.c_str());
    sensor_type = ORB_SLAM3::System::MONOCULAR;
  }

  try {
    RCLCPP_INFO(get_logger(), "Instantiating ORB-SLAM3 System...");
    slam_system_ = std::make_unique<ORB_SLAM3::System>(
      vocabulary_file_,
      settings_file_,
      sensor_type,
      use_viewer_);
    slam_initialized_ = true;
    RCLCPP_INFO(get_logger(), "ORB-SLAM3 System successfully initialized.");
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Exception initializing ORB-SLAM3 System: %s", e.what());
    slam_initialized_ = false;
  } catch (...) {
    RCLCPP_ERROR(get_logger(), "Unknown error initializing ORB-SLAM3 System.");
    slam_initialized_ = false;
  }
}

void ORBSLAM3Node::image_callback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
  // Sequential processing: lock mutex to prevent concurrent TrackMonocular calls
  std::lock_guard<std::mutex> lock(mutex_track_);

  if (!slam_initialized_ || !slam_system_ || slam_system_->isShutDown()) {
    std_msgs::msg::Bool status_msg;
    status_msg.data = false;
    status_pub_->publish(status_msg);
    return;
  }

  // Extract ROS image timestamp converted to seconds (do not use system time)
  double timestamp = msg->header.stamp.sec + static_cast<double>(msg->header.stamp.nanosec) * 1e-9;

  // Convert ROS Image to cv::Mat without unnecessary memory copying
  cv_bridge::CvImageConstPtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
  } catch (const cv_bridge::Exception & e) {
    RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
    std_msgs::msg::Bool status_msg;
    status_msg.data = false;
    status_pub_->publish(status_msg);
    return;
  }

  if (!cv_ptr || cv_ptr->image.empty()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Received empty camera frame.");
    std_msgs::msg::Bool status_msg;
    status_msg.data = false;
    status_pub_->publish(status_msg);
    return;
  }

  // Pass image to ORB-SLAM3 monocular tracking
  Sophus::SE3f Tcw = slam_system_->TrackMonocular(cv_ptr->image, timestamp);
  int tracking_state = slam_system_->GetTrackingState();

  bool is_tracking = (tracking_state == ORB_SLAM3::Tracking::OK || tracking_state == ORB_SLAM3::Tracking::OK_KLT);

  // Publish tracking status: true if tracked, false if lost or not initialized
  std_msgs::msg::Bool status_msg;
  status_msg.data = is_tracking;
  status_pub_->publish(status_msg);

  if (!is_tracking) {
    has_prev_pose_ = false;
    return;
  }

  /*
   * ===========================================================================
   * POSE CONVENTION AND TRANSFORMATION EXPLANATION
   * ===========================================================================
   * ORB-SLAM3 TrackMonocular returns Sophus::SE3f Tcw.
   * Tcw represents the rigid-body transformation from the World frame to the
   * Camera optical frame:
   *   p_c = R_cw * p_w + t_cw
   *
   * In ROS, geometry_msgs/PoseStamped and nav_msgs/Odometry express the pose of
   * the camera frame relative to the world reference frame (Twc):
   *   p_w = R_wc * p_c + t_wc, where Twc = Tcw.inverse()
   *
   * We compute Twc = Tcw.inverse(), extract translation t_wc and rotation
   * matrix R_wc, convert R_wc to a normalized Eigen::Quaternionf satisfying:
   *   x² + y² + z² + w² = 1.0
   *
   * The frame ID is set to camera_frame_ ("camera_optical_frame").
   * This is explicitly the CAMERA optical pose (not drone base_link).
   * ===========================================================================
   */
  Sophus::SE3f Twc = Tcw.inverse();
  Eigen::Vector3f t = Twc.translation();
  Eigen::Quaternionf q = Twc.unit_quaternion();
  q.normalize();

  // Validate finite coordinates and normalized quaternion
  if (!std::isfinite(t.x()) || !std::isfinite(t.y()) || !std::isfinite(t.z()) ||
      !std::isfinite(q.x()) || !std::isfinite(q.y()) || !std::isfinite(q.z()) || !std::isfinite(q.w()) ||
      std::abs(q.norm() - 1.0f) > 0.05f)
  {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Non-finite values or invalid quaternion in pose.");
    has_prev_pose_ = false;
    return;
  }

  // Populate and publish /orbslam/pose (geometry_msgs/msg/PoseStamped)
  geometry_msgs::msg::PoseStamped pose_msg;
  pose_msg.header.stamp = msg->header.stamp;
  pose_msg.header.frame_id = camera_frame_;
  pose_msg.pose.position.x = t.x();
  pose_msg.pose.position.y = t.y();
  pose_msg.pose.position.z = t.z();
  pose_msg.pose.orientation.x = q.x();
  pose_msg.pose.orientation.y = q.y();
  pose_msg.pose.orientation.z = q.z();
  pose_msg.pose.orientation.w = q.w();
  pose_pub_->publish(pose_msg);

  // Populate and publish /orbslam/odometry (nav_msgs/msg/Odometry)
  nav_msgs::msg::Odometry odom_msg;
  odom_msg.header.stamp = msg->header.stamp;
  odom_msg.header.frame_id = camera_frame_;
  odom_msg.child_frame_id = camera_frame_;
  odom_msg.pose.pose = pose_msg.pose;

  for (int i = 0; i < 36; ++i) {
    odom_msg.pose.covariance[i] = 0.0;
    odom_msg.twist.covariance[i] = 0.0;
  }
  odom_msg.pose.covariance[0] = 0.01;   // x
  odom_msg.pose.covariance[7] = 0.01;   // y
  odom_msg.pose.covariance[14] = 0.01;  // z
  odom_msg.pose.covariance[21] = 0.05;  // roll
  odom_msg.pose.covariance[28] = 0.05;  // pitch
  odom_msg.pose.covariance[35] = 0.05;  // yaw

  // Compute linear velocity if previous pose is available
  if (has_prev_pose_ && (timestamp > prev_timestamp_)) {
    double dt = timestamp - prev_timestamp_;
    if (dt > 1e-5) {
      odom_msg.twist.twist.linear.x = (t.x() - prev_x_) / dt;
      odom_msg.twist.twist.linear.y = (t.y() - prev_y_) / dt;
      odom_msg.twist.twist.linear.z = (t.z() - prev_z_) / dt;
      odom_msg.twist.covariance[0] = 0.02;
      odom_msg.twist.covariance[7] = 0.02;
      odom_msg.twist.covariance[14] = 0.02;
    }
  }

  prev_x_ = t.x();
  prev_y_ = t.y();
  prev_z_ = t.z();
  prev_timestamp_ = timestamp;
  has_prev_pose_ = true;

  odom_pub_->publish(odom_msg);
}

}  // namespace orbslam3_wrapper

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<orbslam3_wrapper::ORBSLAM3Node>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
