#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include "drone_interfaces/msg/drone_state.hpp"

namespace drone_state_estimation
{

class StateEstimatorNode : public rclcpp::Node
{
public:
  StateEstimatorNode()
  : Node("state_estimator_node")
  {
    // -------------------------------------------------------------------------
    // Parameters
    // -------------------------------------------------------------------------
    declare_parameter<std::string>("state_frame_id", "odom");
    declare_parameter<std::string>("base_frame_id", "base_link");
    declare_parameter<double>("publish_rate_hz", 50.0);
    declare_parameter<bool>("enable_sanity_checks", true);

    state_frame_id_ = get_parameter("state_frame_id").as_string();
    base_frame_id_ = get_parameter("base_frame_id").as_string();
    enable_sanity_checks_ = get_parameter("enable_sanity_checks").as_bool();

    // -------------------------------------------------------------------------
    // Publishers
    // -------------------------------------------------------------------------
    drone_odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/drone/odometry", 10);
    drone_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/drone/pose", 10);
    drone_twist_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("/drone/twist", 10);
    drone_state_pub_ = create_publisher<drone_interfaces::msg::DroneState>("/drone/state", 10);

    // -------------------------------------------------------------------------
    // Subscribers
    // -------------------------------------------------------------------------
    fusion_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/fusion/odometry", 10,
      std::bind(&StateEstimatorNode::fusion_callback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Drone State Estimation Node initialized.");
  }

private:
  void fusion_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    // -------------------------------------------------------------------------
    // TODO: State Estimation, Validation, and Dissemination
    // -------------------------------------------------------------------------
    // TODO:
    //   1. Perform sanity checks (divergence, NaN detection, variance thresholds).
    //   2. Forward verified /drone/odometry:
    //      - msg->header.frame_id = state_frame_id_;
    //      - msg->child_frame_id = base_frame_id_;
    //      - drone_odom_pub_->publish(*msg);
    //   3. Extract and publish /drone/pose (geometry_msgs::msg::PoseStamped):
    //      - pose_msg.header = msg->header;
    //      - pose_msg.pose = msg->pose.pose;
    //      - drone_pose_pub_->publish(pose_msg);
    //   4. Extract and publish /drone/twist (geometry_msgs::msg::TwistStamped):
    //      - twist_msg.header = msg->header;
    //      - twist_msg.twist = msg->twist.twist;
    //      - drone_twist_pub_->publish(twist_msg);
    //   5. Aggregate into /drone/state (drone_interfaces::msg::DroneState):
    //      - state_msg.position = msg->pose.pose.position;
    //      - state_msg.orientation = msg->pose.pose.orientation;
    //      - state_msg.velocity = msg->twist.twist.linear;
    //      - state_msg.tracking_status = "OK";
    //      - drone_state_pub_->publish(state_msg);
    (void)msg;
  }

  std::string state_frame_id_;
  std::string base_frame_id_;
  bool enable_sanity_checks_{true};

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr fusion_odom_sub_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr drone_odom_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr drone_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr drone_twist_pub_;
  rclcpp::Publisher<drone_interfaces::msg::DroneState>::SharedPtr drone_state_pub_;
};

}  // namespace drone_state_estimation

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<drone_state_estimation::StateEstimatorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
