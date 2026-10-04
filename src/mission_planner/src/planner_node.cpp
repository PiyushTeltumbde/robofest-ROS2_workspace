#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "drone_interfaces/msg/detection.hpp"

namespace mission_planner
{

enum class MissionState
{
  IDLE,
  TAKEOFF,
  SEARCH_COVERAGE,
  INSPECT_SURVIVOR,
  RETURN_TO_LAUNCH,
  EMERGENCY_LAND
};

class PlannerNode : public rclcpp::Node
{
public:
  PlannerNode()
  : Node("planner_node")
  , current_state_(MissionState::IDLE)
  {
    // -------------------------------------------------------------------------
    // Parameters
    // -------------------------------------------------------------------------
    declare_parameter<std::string>("planner_type", "RRT_STAR");
    declare_parameter<double>("max_velocity_mps", 3.0);
    declare_parameter<double>("max_acceleration_mps2", 1.5);
    declare_parameter<double>("safety_margin_m", 0.8);
    declare_parameter<double>("replan_frequency_hz", 2.0);
    declare_parameter<std::string>("search_pattern", "LAWNMOWER");

    planner_type_ = get_parameter("planner_type").as_string();
    max_velocity_ = get_parameter("max_velocity_mps").as_double();
    safety_margin_ = get_parameter("safety_margin_m").as_double();
    replan_freq_ = get_parameter("replan_frequency_hz").as_double();

    RCLCPP_INFO(get_logger(), "Initializing SAR Mission Planner (%s, max_vel=%.1f m/s, margin=%.2f m)...",
      planner_type_.c_str(), max_velocity_, safety_margin_);

    // -------------------------------------------------------------------------
    // Publishers (Commands to ArduPilot and Visualized Paths)
    // -------------------------------------------------------------------------
    cmd_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/ardupilot/cmd_pose", 10);
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/ardupilot/cmd_vel", 10);
    path_pub_ = create_publisher<nav_msgs::msg::Path>("/planner/trajectory", 10);

    // -------------------------------------------------------------------------
    // Subscribers (State feedback, environment map, and target detections)
    // -------------------------------------------------------------------------
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/drone/odometry", 10,
      std::bind(&PlannerNode::odom_callback, this, std::placeholders::_1));

    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/map/pointcloud", 10,
      std::bind(&PlannerNode::cloud_callback, this, std::placeholders::_1));

    occupancy_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map/occupancy", 10,
      std::bind(&PlannerNode::occupancy_callback, this, std::placeholders::_1));

    detection_sub_ = create_subscription<drone_interfaces::msg::Detection>(
      "/detections", 10,
      std::bind(&PlannerNode::detection_callback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Mission Planner skeleton initialized (idle mode).");
  }

private:
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    current_odom_ = msg;
  }

  void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    (void)msg;
  }

  void occupancy_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    // -------------------------------------------------------------------------
    // TODO: Map Ingestion for Collision Checking
    // -------------------------------------------------------------------------
    // Store 2D/3D costmap for RRT*/A* collision avoidance checks.
    (void)msg;
  }

  void detection_callback(const drone_interfaces::msg::Detection::SharedPtr msg)
  {
    // -------------------------------------------------------------------------
    // TODO: Search-and-Rescue Target Response
    // -------------------------------------------------------------------------
    // When survivor or beacon is detected:
    //   1. Log target location and confidence score.
    //   2. Transition mission state: SEARCH_COVERAGE -> INSPECT_SURVIVOR.
    //   3. Compute inspection loiter orbit around target coordinate.
    //   4. Dispatch command waypoints to ArduPilot.
    RCLCPP_INFO(get_logger(), "Target detected: %s (confidence: %.2f)",
      msg->class_name.c_str(), msg->confidence);
  }

  // ---------------------------------------------------------------------------
  // TODO: Path Planning and Trajectory Optimization
  // ---------------------------------------------------------------------------
  // TODO:
  //   1. Implement 3D Kinodynamic RRT* or Jump Point Search in occupancy space.
  //   2. Smooth waypoints using minimum-snap polynomial or B-spline trajectories.
  //   3. Generate time-parameterized velocity / pose setpoints respecting
  //      max_velocity_ and max_acceleration limits.
  //   4. Publish waypoints to /ardupilot/cmd_pose or feed velocity vectors to
  //      /ardupilot/cmd_vel.

  std::string planner_type_;
  double max_velocity_{3.0};
  double safety_margin_{0.8};
  double replan_freq_{2.0};
  MissionState current_state_;

  nav_msgs::msg::Odometry::SharedPtr current_odom_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr cmd_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_sub_;
  rclcpp::Subscription<drone_interfaces::msg::Detection>::SharedPtr detection_sub_;
};

}  // namespace mission_planner

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<mission_planner::PlannerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
