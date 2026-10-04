#include "mapping_3d/mapping_node.hpp"

#include <algorithm>
#include <chrono>
#include <limits>
#include <vector>

#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/imgproc.hpp>

namespace mapping_3d
{

MappingNode::MappingNode(const rclcpp::NodeOptions & options)
: Node("mapping_node", options)
{
  declare_and_get_parameters();

  // Initialize TF2 buffer and listener
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Publishers
  map_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
    "/mapping/point_cloud", 10);
  current_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
    "/mapping/current_cloud", 10);
  status_pub_ = create_publisher<std_msgs::msg::Bool>(
    "/mapping/status", 10);

  // Subscribers
  camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
    camera_info_topic_, 10,
    std::bind(&MappingNode::camera_info_callback, this, std::placeholders::_1));

  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    pose_topic_, 20,
    std::bind(&MappingNode::odom_callback, this, std::placeholders::_1));

  depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
    depth_topic_, 10,
    std::bind(&MappingNode::depth_callback, this, std::placeholders::_1));

  // Periodic publishing timer
  double timer_period_sec = (publish_rate_hz_ > 0.0) ? (1.0 / publish_rate_hz_) : 0.2;
  publish_timer_ = create_wall_timer(
    std::chrono::duration<double>(timer_period_sec),
    std::bind(&MappingNode::publish_timer_callback, this));

  RCLCPP_INFO(get_logger(), "MappingNode successfully initialized.");
  RCLCPP_INFO(get_logger(), "Listening for depth on: %s", depth_topic_.c_str());
  RCLCPP_INFO(get_logger(), "Listening for camera calibration on: %s", camera_info_topic_.c_str());
  RCLCPP_INFO(get_logger(), "Listening for odometry on: %s", pose_topic_.c_str());
  RCLCPP_INFO(get_logger(), "Target map frame: %s | Camera frame: %s | Base frame: %s",
    map_frame_.c_str(), camera_frame_.c_str(), base_frame_.c_str());
}

void MappingNode::declare_and_get_parameters()
{
  declare_parameter<std::string>("pose_topic", "/drone/odometry");
  declare_parameter<std::string>("depth_topic", "/camera/depth/image_raw");
  declare_parameter<std::string>("camera_info_topic", "/camera/camera_info");
  declare_parameter<std::string>("map_frame", "map");
  declare_parameter<std::string>("camera_frame", "camera_optical_frame");
  declare_parameter<std::string>("base_frame", "base_link");
  declare_parameter<double>("min_depth_m", 0.2);
  declare_parameter<double>("max_depth_m", 8.0);
  declare_parameter<int>("pixel_stride", 4);
  declare_parameter<double>("voxel_size_m", 0.05);
  declare_parameter<int>("max_points", 200000);
  declare_parameter<int>("max_map_points", 1000000);
  declare_parameter<double>("max_pose_age_sec", 0.2);
  declare_parameter<double>("publish_rate_hz", 5.0);

  pose_topic_ = get_parameter("pose_topic").as_string();
  depth_topic_ = get_parameter("depth_topic").as_string();
  camera_info_topic_ = get_parameter("camera_info_topic").as_string();
  map_frame_ = get_parameter("map_frame").as_string();
  camera_frame_ = get_parameter("camera_frame").as_string();
  base_frame_ = get_parameter("base_frame").as_string();
  min_depth_m_ = get_parameter("min_depth_m").as_double();
  max_depth_m_ = get_parameter("max_depth_m").as_double();
  pixel_stride_ = get_parameter("pixel_stride").as_int();
  voxel_size_m_ = get_parameter("voxel_size_m").as_double();
  max_points_ = get_parameter("max_points").as_int();
  max_map_points_ = get_parameter("max_map_points").as_int();
  max_pose_age_sec_ = get_parameter("max_pose_age_sec").as_double();
  publish_rate_hz_ = get_parameter("publish_rate_hz").as_double();

  // Validate parameters
  if (min_depth_m_ < 0.0) {
    RCLCPP_WARN(get_logger(), "min_depth_m cannot be negative, resetting to 0.1");
    min_depth_m_ = 0.1;
  }
  if (max_depth_m_ <= min_depth_m_) {
    RCLCPP_WARN(get_logger(), "max_depth_m <= min_depth_m, resetting max_depth_m to 8.0");
    max_depth_m_ = 8.0;
  }
  if (pixel_stride_ < 1) {
    RCLCPP_WARN(get_logger(), "pixel_stride < 1, resetting to 1");
    pixel_stride_ = 1;
  }
  if (voxel_size_m_ <= 0.0) {
    RCLCPP_WARN(get_logger(), "voxel_size_m <= 0, resetting to 0.05");
    voxel_size_m_ = 0.05;
  }
  if (max_map_points_ < 1000) {
    max_map_points_ = 1000;
  }
  if (max_pose_age_sec_ <= 0.0) {
    max_pose_age_sec_ = 0.2;
  }

  RCLCPP_INFO(get_logger(), "Loaded MappingNode configuration:");
  RCLCPP_INFO(get_logger(), "  pose_topic:        %s", pose_topic_.c_str());
  RCLCPP_INFO(get_logger(), "  depth_topic:       %s", depth_topic_.c_str());
  RCLCPP_INFO(get_logger(), "  camera_info_topic: %s", camera_info_topic_.c_str());
  RCLCPP_INFO(get_logger(), "  map_frame:         %s", map_frame_.c_str());
  RCLCPP_INFO(get_logger(), "  camera_frame:      %s", camera_frame_.c_str());
  RCLCPP_INFO(get_logger(), "  base_frame:        %s", base_frame_.c_str());
  RCLCPP_INFO(get_logger(), "  min_depth_m:       %.2f", min_depth_m_);
  RCLCPP_INFO(get_logger(), "  max_depth_m:       %.2f", max_depth_m_);
  RCLCPP_INFO(get_logger(), "  pixel_stride:      %d", pixel_stride_);
  RCLCPP_INFO(get_logger(), "  voxel_size_m:      %.3f", voxel_size_m_);
  RCLCPP_INFO(get_logger(), "  max_points:        %d", max_points_);
  RCLCPP_INFO(get_logger(), "  max_map_points:    %d", max_map_points_);
  RCLCPP_INFO(get_logger(), "  max_pose_age_sec:  %.3f", max_pose_age_sec_);
  RCLCPP_INFO(get_logger(), "  publish_rate_hz:   %.1f", publish_rate_hz_);
}

void MappingNode::camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
  double fx = msg->k[0];
  double fy = msg->k[4];
  double cx = msg->k[2];
  double cy = msg->k[5];

  // If intrinsic matrix K has zeros, check projection matrix P
  if (fx <= 0.0 || fy <= 0.0) {
    fx = msg->p[0];
    fy = msg->p[5];
    cx = msg->p[2];
    cy = msg->p[6];
  }

  if (fx > 0.0 && fy > 0.0 && std::isfinite(fx) && std::isfinite(fy) &&
      std::isfinite(cx) && std::isfinite(cy))
  {
    std::lock_guard<std::mutex> lock(cam_info_mutex_);
    fx_ = fx;
    fy_ = fy;
    cx_ = cx;
    cy_ = cy;
    if (!has_camera_info_) {
      has_camera_info_ = true;
      RCLCPP_INFO(get_logger(),
        "Camera calibration received: fx=%.2f, fy=%.2f, cx=%.2f, cy=%.2f",
        fx_, fy_, cx_, cy_);
    }
  } else {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
      "Received CameraInfo with invalid intrinsics (fx=%.2f, fy=%.2f). Waiting for valid camera calibration...",
      fx, fy);
  }
}

void MappingNode::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(odom_mutex_);
  has_odom_ = true;
  last_odom_stamp_ = msg->header.stamp;
  pose_history_.push_back(*msg);

  // Keep pose buffer bounded
  while (pose_history_.size() > 500) {
    pose_history_.pop_front();
  }

  // Prune entries older than 5.0 seconds compared to latest received pose
  rclcpp::Time latest_time = msg->header.stamp;
  while (!pose_history_.empty()) {
    rclcpp::Time oldest_time = pose_history_.front().header.stamp;
    if ((latest_time - oldest_time).seconds() > 5.0) {
      pose_history_.pop_front();
    } else {
      break;
    }
  }
}

bool MappingNode::find_closest_pose(
  const rclcpp::Time & depth_stamp,
  nav_msgs::msg::Odometry & closest_odom)
{
  std::lock_guard<std::mutex> lock(odom_mutex_);
  if (pose_history_.empty()) {
    return false;
  }

  double min_dt = std::numeric_limits<double>::max();
  size_t best_idx = 0;

  for (size_t i = 0; i < pose_history_.size(); ++i) {
    double dt = std::abs((depth_stamp - rclcpp::Time(pose_history_[i].header.stamp)).seconds());
    if (dt < min_dt) {
      min_dt = dt;
      best_idx = i;
    }
  }

  if (min_dt <= max_pose_age_sec_) {
    closest_odom = pose_history_[best_idx];
    return true;
  }

  return false;
}

bool MappingNode::compute_camera_to_map_transform(
  const nav_msgs::msg::Odometry & odom,
  const std::string & depth_camera_frame,
  tf2::Transform & T_map_cam)
{
  std::string cam_frame = depth_camera_frame.empty() ? camera_frame_ : depth_camera_frame;
  std::string odom_child = odom.child_frame_id.empty() ? base_frame_ : odom.child_frame_id;
  std::string odom_parent = odom.header.frame_id.empty() ? map_frame_ : odom.header.frame_id;

  // 1. Transform from camera optical frame to odom child frame (e.g. base_link)
  tf2::Transform T_child_cam;
  T_child_cam.setIdentity();

  if (odom_child != cam_frame) {
    try {
      geometry_msgs::msg::TransformStamped tf_child_cam =
        tf_buffer_->lookupTransform(odom_child, cam_frame, tf2::TimePointZero);
      tf2::fromMsg(tf_child_cam.transform, T_child_cam);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "Waiting for TF: transform from '%s' to '%s' unavailable: %s",
        cam_frame.c_str(), odom_child.c_str(), ex.what());
      return false;
    }
  }

  // 2. Transform from odom child frame to odom parent frame (from Odometry pose)
  tf2::Transform T_parent_child;
  tf2::fromMsg(odom.pose.pose, T_parent_child);

  // 3. Transform from odom parent frame to target map_frame_
  tf2::Transform T_map_parent;
  T_map_parent.setIdentity();

  if (odom_parent == map_frame_) {
    T_map_parent.setIdentity();
  } else if (odom_parent == cam_frame && odom_child == cam_frame) {
    // When using ORB-SLAM3 odometry, header and child frame are camera_optical_frame,
    // and pose is camera pose relative to the world map reference frame directly.
    T_map_parent.setIdentity();
  } else {
    try {
      geometry_msgs::msg::TransformStamped tf_map_parent =
        tf_buffer_->lookupTransform(map_frame_, odom_parent, tf2::TimePointZero);
      tf2::fromMsg(tf_map_parent.transform, T_map_parent);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "Waiting for TF: transform from '%s' to '%s' unavailable: %s",
        odom_parent.c_str(), map_frame_.c_str(), ex.what());
      return false;
    }
  }

  // Total rigid body transform: camera_optical_frame -> map
  T_map_cam = T_map_parent * T_parent_child * T_child_cam;
  return true;
}

void MappingNode::depth_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  {
    std::lock_guard<std::mutex> lock(depth_state_mutex_);
    has_depth_ = true;
    last_depth_stamp_ = msg->header.stamp;
  }

  // 1. Camera calibration verification
  double fx = 0.0, fy = 0.0, cx = 0.0, cy = 0.0;
  {
    std::lock_guard<std::mutex> lock(cam_info_mutex_);
    if (!has_camera_info_ || fx_ <= 0.0 || fy_ <= 0.0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "Waiting for valid camera calibration...");
      return;
    }
    fx = fx_;
    fy = fy_;
    cx = cx_;
    cy = cy_;
  }

  // 2. Odometry pose synchronization
  nav_msgs::msg::Odometry closest_odom;
  rclcpp::Time depth_stamp = msg->header.stamp;
  if (!find_closest_pose(depth_stamp, closest_odom)) {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    if (pose_history_.empty()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "Waiting for odometry...");
    } else {
      double dt = std::abs((depth_stamp - rclcpp::Time(pose_history_.back().header.stamp)).seconds());
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "No sufficiently recent pose found for depth frame (closest age: %.3fs > max_pose_age_sec: %.3fs). Skipping frame.",
        dt, max_pose_age_sec_);
    }
    return;
  }

  // 3. TF2 coordinate transformation computation
  tf2::Transform T_map_cam;
  if (!compute_camera_to_map_transform(closest_odom, msg->header.frame_id, T_map_cam)) {
    return;
  }

  // 4. Depth image extraction and unit normalization (to meters)
  cv::Mat depth_image;
  try {
    if (msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1 || msg->encoding == "mono16") {
      cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, msg->encoding);
      cv_ptr->image.convertTo(depth_image, CV_32F, 0.001);  // Convert mm to meters
    } else if (msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
      cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::TYPE_32FC1);
      depth_image = cv_ptr->image;  // Already float meters
    } else {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "Unsupported depth image encoding '%s'. Expected '16UC1' or '32FC1'.", msg->encoding.c_str());
      return;
    }
  } catch (const cv_bridge::Exception & e) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
      "cv_bridge exception: %s", e.what());
    return;
  }

  if (depth_image.empty()) {
    return;
  }

  // 5. 3D point projection and map-frame transformation
  int stride = std::max(1, pixel_stride_);
  std::vector<Point3D> current_points;
  current_points.reserve(std::min(static_cast<size_t>(max_points_),
    static_cast<size_t>((depth_image.rows / stride) * (depth_image.cols / stride))));

  for (int v = 0; v < depth_image.rows; v += stride) {
    const float * row = depth_image.ptr<float>(v);
    for (int u = 0; u < depth_image.cols; u += stride) {
      float z = row[u];
      if (!std::isfinite(z) || z <= 0.0f || z < min_depth_m_ || z > max_depth_m_) {
        continue;
      }
      float x = (static_cast<float>(u) - cx) * z / fx;
      float y = (static_cast<float>(v) - cy) * z / fy;

      // Point in camera_optical_frame
      tf2::Vector3 pt_cam(x, y, z);
      // Transform into target map frame via TF2
      tf2::Vector3 pt_map = T_map_cam * pt_cam;

      if (!std::isfinite(pt_map.x()) || !std::isfinite(pt_map.y()) || !std::isfinite(pt_map.z())) {
        continue;
      }

      Point3D pt;
      pt.x = static_cast<float>(pt_map.x());
      pt.y = static_cast<float>(pt_map.y());
      pt.z = static_cast<float>(pt_map.z());
      pt.seq = 0;
      current_points.push_back(pt);

      if (static_cast<int>(current_points.size()) >= max_points_) {
        break;
      }
    }
    if (static_cast<int>(current_points.size()) >= max_points_) {
      break;
    }
  }

  // 6. Publish /mapping/current_cloud
  sensor_msgs::msg::PointCloud2 current_cloud_msg =
    create_cloud_msg(current_points, msg->header.stamp, map_frame_);
  current_cloud_pub_->publish(current_cloud_msg);

  // 7. Accumulate points into global voxel map
  add_points_to_global_map(current_points);
}

void MappingNode::add_points_to_global_map(const std::vector<Point3D> & points)
{
  if (points.empty()) {
    return;
  }

  std::lock_guard<std::mutex> lock(map_mutex_);

  for (const auto & pt : points) {
    int64_t vx = static_cast<int64_t>(std::floor(pt.x / voxel_size_m_));
    int64_t vy = static_cast<int64_t>(std::floor(pt.y / voxel_size_m_));
    int64_t vz = static_cast<int64_t>(std::floor(pt.z / voxel_size_m_));
    VoxelCoord key{vx, vy, vz};

    auto it = voxel_map_.find(key);
    if (it == voxel_map_.end()) {
      voxel_map_.emplace(key, Point3D{pt.x, pt.y, pt.z, ++point_sequence_});
    } else {
      it->second.seq = ++point_sequence_;
    }
  }

  // Enforce bounded map size: downsample oldest/redundant voxels if size limit exceeded
  if (voxel_map_.size() > static_cast<size_t>(max_map_points_)) {
    size_t target_size = static_cast<size_t>(max_map_points_ * 0.8);
    size_t num_to_remove = voxel_map_.size() - target_size;

    std::vector<uint64_t> sequences;
    sequences.reserve(voxel_map_.size());
    for (const auto & kv : voxel_map_) {
      sequences.push_back(kv.second.seq);
    }

    std::nth_element(sequences.begin(), sequences.begin() + num_to_remove, sequences.end());
    uint64_t cutoff_seq = sequences[num_to_remove];

    for (auto it = voxel_map_.begin(); it != voxel_map_.end();) {
      if (it->second.seq <= cutoff_seq) {
        it = voxel_map_.erase(it);
      } else {
        ++it;
      }
    }
  }
}

sensor_msgs::msg::PointCloud2 MappingNode::create_cloud_msg(
  const std::vector<Point3D> & points,
  const rclcpp::Time & stamp,
  const std::string & frame_id)
{
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.stamp = stamp;
  cloud.header.frame_id = frame_id;
  cloud.height = 1;
  cloud.width = static_cast<uint32_t>(points.size());
  cloud.is_dense = true;
  cloud.is_bigendian = false;

  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(points.size());

  if (!points.empty()) {
    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");

    for (const auto & pt : points) {
      *iter_x = pt.x;
      *iter_y = pt.y;
      *iter_z = pt.z;
      ++iter_x;
      ++iter_y;
      ++iter_z;
    }
  }

  return cloud;
}

void MappingNode::publish_timer_callback()
{
  rclcpp::Time current_time = now();

  // Print throttled waiting messages in offline/idle mode (Section 28)
  {
    std::lock_guard<std::mutex> lock(depth_state_mutex_);
    if (!has_depth_) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
        "Waiting for depth image on '%s'...", depth_topic_.c_str());
    }
  }
  {
    std::lock_guard<std::mutex> lock(cam_info_mutex_);
    if (!has_camera_info_) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
        "Waiting for camera calibration on '%s'...", camera_info_topic_.c_str());
    }
  }
  {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    if (!has_odom_) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
        "Waiting for odometry on '%s'...", pose_topic_.c_str());
    }
  }

  // Active status requires valid calibration and recent depth and odometry streams
  bool active = false;
  {
    std::lock_guard<std::mutex> cam_lock(cam_info_mutex_);
    std::lock_guard<std::mutex> depth_lock(depth_state_mutex_);
    std::lock_guard<std::mutex> odom_lock(odom_mutex_);

    bool calib_ok = has_camera_info_ && (fx_ > 0.0) && (fy_ > 0.0);
    bool depth_ok = has_depth_ && (std::abs((current_time - last_depth_stamp_).seconds()) < 2.0);
    bool odom_ok = has_odom_ && (std::abs((current_time - last_odom_stamp_).seconds()) < 2.0);

    active = calib_ok && depth_ok && odom_ok;
  }

  std_msgs::msg::Bool status_msg;
  status_msg.data = active;
  status_pub_->publish(status_msg);

  // Publish accumulated global point cloud
  std::vector<Point3D> map_points;
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    map_points.reserve(voxel_map_.size());
    for (const auto & kv : voxel_map_) {
      map_points.push_back(kv.second);
    }
  }

  sensor_msgs::msg::PointCloud2 global_cloud =
    create_cloud_msg(map_points, current_time, map_frame_);
  map_cloud_pub_->publish(global_cloud);
}

}  // namespace mapping_3d

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<mapping_3d::MappingNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
