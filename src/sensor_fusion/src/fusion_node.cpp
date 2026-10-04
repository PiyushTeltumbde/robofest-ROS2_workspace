#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>

#include "sensor_fusion/ekf.hpp"

namespace sensor_fusion
{

class FusionNode : public rclcpp::Node
{
public:
  FusionNode()
  : Node("fusion_node")
  {
    // -------------------------------------------------------------------------
    // Parameters
    // -------------------------------------------------------------------------
    declare_parameter<double>("process_noise_accel", 0.01);
    declare_parameter<double>("process_noise_gyro", 0.001);
    declare_parameter<double>("measurement_noise_vo", 0.05);
    declare_parameter<double>("measurement_noise_gps", 1.5);
    declare_parameter<double>("measurement_noise_baro", 0.2);
    declare_parameter<bool>("use_imu", true);
    declare_parameter<bool>("use_gps", true);
    declare_parameter<bool>("use_visual_odometry", true);

    double q_accel = get_parameter("process_noise_accel").as_double();
    double q_gyro = get_parameter("process_noise_gyro").as_double();
    double r_vo = get_parameter("measurement_noise_vo").as_double();
    double r_gps = get_parameter("measurement_noise_gps").as_double();
    double r_baro = get_parameter("measurement_noise_baro").as_double();
    bool use_imu = get_parameter("use_imu").as_bool();
    bool use_gps = get_parameter("use_gps").as_bool();
    bool use_vo = get_parameter("use_visual_odometry").as_bool();

    ekf_ = std::make_unique<ExtendedKalmanFilter>();
    ekf_->configure(q_accel, q_gyro, r_vo, r_gps, r_baro, use_imu, use_gps, use_vo);

    // -------------------------------------------------------------------------
    // Publishers
    // -------------------------------------------------------------------------
    fused_odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/fusion/odometry", 10);

    // -------------------------------------------------------------------------
    // Subscribers
    // -------------------------------------------------------------------------
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "/imu/data", 50,
      std::bind(&FusionNode::imu_callback, this, std::placeholders::_1));

    vo_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/orbslam/odometry", 10,
      std::bind(&FusionNode::vo_callback, this, std::placeholders::_1));

    gps_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
      "/gps/fix", 10,
      std::bind(&FusionNode::gps_callback, this, std::placeholders::_1));

    baro_sub_ = create_subscription<sensor_msgs::msg::FluidPressure>(
      "/barometer/data", 10,
      std::bind(&FusionNode::baro_callback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Sensor Fusion (EKF) Node initialized.");
  }

private:
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    // -------------------------------------------------------------------------
    // TODO: IMU Prediction Step Integration
    // -------------------------------------------------------------------------
    // Calculate dt from last IMU timestamp and call ekf_->predict(dt, *msg);
    // Publish high-rate fused odometry output if filter is initialized.
    (void)msg;
  }

  void vo_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    // -------------------------------------------------------------------------
    // TODO: Visual Odometry Update Step Integration
    // -------------------------------------------------------------------------
    // Validate timestamp and covariance, call ekf_->update_vo(*msg);
    (void)msg;
  }

  void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
  {
    // -------------------------------------------------------------------------
    // TODO: GPS Update Step Integration
    // -------------------------------------------------------------------------
    // Verify valid fix status, call ekf_->update_gps(*msg);
    (void)msg;
  }

  void baro_callback(const sensor_msgs::msg::FluidPressure::SharedPtr msg)
  {
    // -------------------------------------------------------------------------
    // TODO: Barometer Update Step Integration
    // -------------------------------------------------------------------------
    // Call ekf_->update_barometer(*msg);
    (void)msg;
  }

  std::unique_ptr<ExtendedKalmanFilter> ekf_;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr vo_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
  rclcpp::Subscription<sensor_msgs::msg::FluidPressure>::SharedPtr baro_sub_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr fused_odom_pub_;
};

}  // namespace sensor_fusion

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<sensor_fusion::FusionNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
