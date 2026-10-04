#ifndef SENSOR_FUSION__EKF_HPP_
#define SENSOR_FUSION__EKF_HPP_

#include <vector>
#include <string>
#include <memory>

#include <geometry_msgs/msg/pose_with_covariance.hpp>
#include <geometry_msgs/msg/twist_with_covariance.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>

namespace sensor_fusion
{

/**
 * @brief State vector indices for 15-state error-state / direct EKF
 * [p_x, p_y, p_z, v_x, v_y, v_z, q_w, q_x, q_y, q_z, bg_x, bg_y, bg_z, ba_x, ba_y, ba_z]
 */
struct EKFState
{
  double x{0.0}, y{0.0}, z{0.0};
  double vx{0.0}, vy{0.0}, vz{0.0};
  double qw{1.0}, qx{0.0}, qy{0.0}, qz{0.0};
  double bgx{0.0}, bgy{0.0}, bgz{0.0};
  double bax{0.0}, bay{0.0}, baz{0.0};
};

/**
 * @brief Extended Kalman Filter class skeleton for fusing IMU, VO, GPS, and Barometer.
 */
class ExtendedKalmanFilter
{
public:
  ExtendedKalmanFilter();
  virtual ~ExtendedKalmanFilter() = default;

  /**
   * @brief Configure filter parameters and noise covariances
   */
  void configure(
    double process_noise_accel,
    double process_noise_gyro,
    double measurement_noise_vo,
    double measurement_noise_gps,
    double measurement_noise_baro,
    bool use_imu,
    bool use_gps,
    bool use_visual_odometry);

  /**
   * @brief Prediction step using IMU high-rate kinematics
   * @param dt Time delta (seconds)
   * @param imu IMU measurement
   */
  void predict(double dt, const sensor_msgs::msg::Imu & imu);

  /**
   * @brief Measurement update using Visual Odometry from ORB-SLAM3
   * @param vo Visual Odometry message
   */
  void update_vo(const nav_msgs::msg::Odometry & vo);

  /**
   * @brief Measurement update using Global Positioning System
   * @param gps GPS NavSatFix message
   */
  void update_gps(const sensor_msgs::msg::NavSatFix & gps);

  /**
   * @brief Measurement update using Barometer pressure altitude
   * @param baro Barometer fluid pressure measurement
   */
  void update_barometer(const sensor_msgs::msg::FluidPressure & baro);

  /**
   * @brief Populate output fused odometry message
   * @param out_odom Output odometry message
   */
  void get_fused_odometry(nav_msgs::msg::Odometry & out_odom) const;

  /**
   * @brief Check whether the filter has initialized with valid reference frames
   */
  bool is_initialized() const;

private:
  bool initialized_{false};

  EKFState state_;

  // Filter configuration flags
  bool use_imu_{true};
  bool use_gps_{true};
  bool use_vo_{true};

  // Noise covariance parameters
  double q_accel_{0.01};
  double q_gyro_{0.001};
  double r_vo_{0.05};
  double r_gps_{1.5};
  double r_baro_{0.2};

  // ---------------------------------------------------------------------------
  // TODO: EKF Matrices Implementation
  // ---------------------------------------------------------------------------
  // In the full implementation:
  //   Eigen::Matrix<double, 15, 15> P_; // Error covariance
  //   Eigen::Matrix<double, 15, 15> Q_; // Process noise covariance
  //   Eigen::Matrix<double, 15, 15> F_; // State transition Jacobian
  //   Eigen::Vector3d gps_datum_;       // GPS reference origin for ENU conversion
};

}  // namespace sensor_fusion

#endif  // SENSOR_FUSION__EKF_HPP_
