#include "sensor_fusion/ekf.hpp"

#include <cmath>
#include <iostream>

namespace sensor_fusion
{

ExtendedKalmanFilter::ExtendedKalmanFilter()
: initialized_(false)
{
}

void ExtendedKalmanFilter::configure(
  double process_noise_accel,
  double process_noise_gyro,
  double measurement_noise_vo,
  double measurement_noise_gps,
  double measurement_noise_baro,
  bool use_imu,
  bool use_gps,
  bool use_visual_odometry)
{
  q_accel_ = process_noise_accel;
  q_gyro_ = process_noise_gyro;
  r_vo_ = measurement_noise_vo;
  r_gps_ = measurement_noise_gps;
  r_baro_ = measurement_noise_baro;
  use_imu_ = use_imu;
  use_gps_ = use_gps;
  use_vo_ = use_visual_odometry;

  std::cout << "[EKF] Configured: use_imu=" << use_imu_
            << ", use_gps=" << use_gps_
            << ", use_vo=" << use_vo_ << std::endl;
}

void ExtendedKalmanFilter::predict(double dt, const sensor_msgs::msg::Imu & imu)
{
  if (!use_imu_) {
    return;
  }

  // ---------------------------------------------------------------------------
  // TODO: EKF Kinematic Propagation / Predict Step
  // ---------------------------------------------------------------------------
  // TODO:
  //   1. Correct raw IMU measurements using estimated biases:
  //      accel_corr = imu.linear_acceleration - state_.ba;
  //      gyro_corr  = imu.angular_velocity - state_.bg;
  //   2. Propagate orientation quaternion:
  //      q_next = q_prev * exp(0.5 * gyro_corr * dt);
  //   3. Rotate corrected acceleration to world (ENU/NED) frame:
  //      a_world = R(q) * accel_corr + gravity_vector;
  //   4. Propagate position and velocity:
  //      p_next = p_prev + v_prev * dt + 0.5 * a_world * dt^2;
  //      v_next = v_prev + a_world * dt;
  //   5. Compute state transition Jacobian F_x and noise Jacobian F_i:
  //      P = F_x * P * F_x^T + F_i * Q * F_i^T;
  // ---------------------------------------------------------------------------
  (void)dt;
  (void)imu;
}

void ExtendedKalmanFilter::update_vo(const nav_msgs::msg::Odometry & vo)
{
  if (!use_vo_) {
    return;
  }

  // ---------------------------------------------------------------------------
  // TODO: Visual Odometry Measurement Update Step
  // ---------------------------------------------------------------------------
  // TODO:
  //   1. Check VO tracking status / covariance health.
  //   2. Align VO coordinate frame with world navigation frame if needed.
  //   3. Compute innovation vector:
  //      y = z_vo - H_vo * x;
  //   4. Innovation covariance:
  //      S = H_vo * P * H_vo^T + R_vo;
  //   5. Kalman Gain:
  //      K = P * H_vo^T * S^-1;
  //   6. State and Covariance correction:
  //      x = x + K * y;
  //      P = (I - K * H_vo) * P;
  // ---------------------------------------------------------------------------
  (void)vo;
}

void ExtendedKalmanFilter::update_gps(const sensor_msgs::msg::NavSatFix & gps)
{
  if (!use_gps_) {
    return;
  }

  // ---------------------------------------------------------------------------
  // TODO: GPS Measurement Update Step
  // ---------------------------------------------------------------------------
  // TODO:
  //   1. Check GPS fix status (STATUS_FIX or RTK).
  //   2. If datum not initialized, set current lat/lon/alt as origin.
  //   3. Convert WGS-84 (lat, lon, alt) to local Cartesian (ENU):
  //      [east, north, up] = geodetic_to_enu(lat, lon, alt, datum);
  //   4. Compute GPS innovation:
  //      y_gps = [east, north] - [state_.x, state_.y];
  //   5. Compute Kalman gain and update horizontal position and velocity.
  // ---------------------------------------------------------------------------
  (void)gps;
}

void ExtendedKalmanFilter::update_barometer(const sensor_msgs::msg::FluidPressure & baro)
{
  // ---------------------------------------------------------------------------
  // TODO: Barometric Altitude Update Step
  // ---------------------------------------------------------------------------
  // TODO:
  //   1. Convert static pressure to altitude:
  //      alt = 44330.0 * (1.0 - pow(pressure / 101325.0, 0.190295));
  //   2. Compute innovation with respect to state_.z:
  //      y_baro = alt - state_.z;
  //   3. Apply scalar Kalman update to vertical position and velocity.
  // ---------------------------------------------------------------------------
  (void)baro;
}

void ExtendedKalmanFilter::get_fused_odometry(nav_msgs::msg::Odometry & out_odom) const
{
  // ---------------------------------------------------------------------------
  // TODO: Output Fused Odometry State
  // ---------------------------------------------------------------------------
  // Populate message header, pose with covariance, twist with covariance
  out_odom.pose.pose.position.x = state_.x;
  out_odom.pose.pose.position.y = state_.y;
  out_odom.pose.pose.position.z = state_.z;
  out_odom.pose.pose.orientation.w = state_.qw;
  out_odom.pose.pose.orientation.x = state_.qx;
  out_odom.pose.pose.orientation.y = state_.qy;
  out_odom.pose.pose.orientation.z = state_.qz;

  out_odom.twist.twist.linear.x = state_.vx;
  out_odom.twist.twist.linear.y = state_.vy;
  out_odom.twist.twist.linear.z = state_.vz;
}

bool ExtendedKalmanFilter::is_initialized() const
{
  return initialized_;
}

}  // namespace sensor_fusion
