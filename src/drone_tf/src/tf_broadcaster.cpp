#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

namespace drone_tf
{

class DroneTFBroadcaster : public rclcpp::Node
{
public:
  DroneTFBroadcaster()
  : Node("drone_tf_broadcaster")
  {
    static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    dynamic_tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

    // Broadcast static extrinsic calibrations of drone physical frame
    publish_static_transforms();

    // -------------------------------------------------------------------------
    // Odometry subscriber for dynamic TF broadcasting
    // -------------------------------------------------------------------------
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/drone/odometry", 10,
      std::bind(&DroneTFBroadcaster::odom_callback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Drone TF Broadcaster initialized.");
    RCLCPP_INFO(get_logger(), "TF Tree Skeleton Prepared: map -> odom -> base_link -> [camera_link -> camera_optical_frame, imu_link]");
  }

private:
  void publish_static_transforms()
  {
    std::vector<geometry_msgs::msg::TransformStamped> static_transforms;
    rclcpp::Time now = get_clock()->now();

    // 1. base_link -> camera_link (forward-facing camera mount offset)
    geometry_msgs::msg::TransformStamped t_cam;
    t_cam.header.stamp = now;
    t_cam.header.frame_id = "base_link";
    t_cam.child_frame_id = "camera_link";
    t_cam.transform.translation.x = 0.10;  // 10 cm ahead of CG
    t_cam.transform.translation.y = 0.0;
    t_cam.transform.translation.z = 0.02;  // 2 cm above CG
    t_cam.transform.rotation.x = 0.0;
    t_cam.transform.rotation.y = 0.0;
    t_cam.transform.rotation.z = 0.0;
    t_cam.transform.rotation.w = 1.0;
    static_transforms.push_back(t_cam);

    // 2. camera_link -> camera_optical_frame (standard ROS optical frame rotation)
    // Converts from ROS body frame (X fwd, Y left, Z up) to optical frame (Z fwd, X right, Y down)
    geometry_msgs::msg::TransformStamped t_opt;
    t_opt.header.stamp = now;
    t_opt.header.frame_id = "camera_link";
    t_opt.child_frame_id = "camera_optical_frame";
    t_opt.transform.translation.x = 0.0;
    t_opt.transform.translation.y = 0.0;
    t_opt.transform.translation.z = 0.0;
    // Quaternion for (-90 deg pitch, 90 deg yaw): [-0.5, 0.5, -0.5, 0.5]
    t_opt.transform.rotation.x = -0.5;
    t_opt.transform.rotation.y = 0.5;
    t_opt.transform.rotation.z = -0.5;
    t_opt.transform.rotation.w = 0.5;
    static_transforms.push_back(t_opt);

    // 3. base_link -> imu_link (rigid IMU placement on flight controller)
    geometry_msgs::msg::TransformStamped t_imu;
    t_imu.header.stamp = now;
    t_imu.header.frame_id = "base_link";
    t_imu.child_frame_id = "imu_link";
    t_imu.transform.translation.x = 0.0;
    t_imu.transform.translation.y = 0.0;
    t_imu.transform.translation.z = 0.0;
    t_imu.transform.rotation.x = 0.0;
    t_imu.transform.rotation.y = 0.0;
    t_imu.transform.rotation.z = 0.0;
    t_imu.transform.rotation.w = 1.0;
    static_transforms.push_back(t_imu);

    static_tf_broadcaster_->sendTransform(static_transforms);
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    // -------------------------------------------------------------------------
    // TODO: Dynamic TF Broadcasting (odom -> base_link)
    // -------------------------------------------------------------------------
    // IMPORTANT: Per system design, do NOT publish fake dynamic transforms.
    // When real sensor data and EKF estimation are active:
    //   geometry_msgs::msg::TransformStamped t;
    //   t.header.stamp = msg->header.stamp;
    //   t.header.frame_id = "odom";
    //   t.child_frame_id = "base_link";
    //   t.transform.translation.x = msg->pose.pose.position.x;
    //   t.transform.translation.y = msg->pose.pose.position.y;
    //   t.transform.translation.z = msg->pose.pose.position.z;
    //   t.transform.rotation = msg->pose.pose.orientation;
    //   dynamic_tf_broadcaster_->sendTransform(t);
    //
    // TODO: Dynamic TF Broadcasting (map -> odom)
    //   Broadcasted when loop-closure drift correction is computed between
    //   global SLAM map frame and local odometry frame.
    (void)msg;
  }

  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> dynamic_tf_broadcaster_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
};

}  // namespace drone_tf

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<drone_tf::DroneTFBroadcaster>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
