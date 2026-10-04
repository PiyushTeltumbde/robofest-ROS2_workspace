#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/string.hpp>

namespace ardupilot_interface
{

class ArduPilotNode : public rclcpp::Node
{
public:
  ArduPilotNode()
  : Node("ardupilot_node")
  {
    // -------------------------------------------------------------------------
    // Parameters
    // -------------------------------------------------------------------------
    declare_parameter<std::string>("connection", "udp:127.0.0.1:14550");
    declare_parameter<int>("baudrate", 921600);
    declare_parameter<int>("system_id", 1);
    declare_parameter<int>("component_id", 1);

    connection_ = get_parameter("connection").as_string();
    baudrate_ = get_parameter("baudrate").as_int();
    system_id_ = get_parameter("system_id").as_int();
    component_id_ = get_parameter("component_id").as_int();

    RCLCPP_INFO(get_logger(), "Initializing ArduPilot Interface Node...");
    RCLCPP_INFO(get_logger(), "  Connection URI: %s", connection_.c_str());
    RCLCPP_INFO(get_logger(), "  Baudrate: %d", baudrate_);
    RCLCPP_INFO(get_logger(), "  SysID: %d, CompID: %d", system_id_, component_id_);

    // -------------------------------------------------------------------------
    // Publishers (Telemetry stream from ArduPilot to ROS 2)
    // -------------------------------------------------------------------------
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("/imu/data", 50);
    gps_pub_ = create_publisher<sensor_msgs::msg::NavSatFix>("/gps/fix", 10);
    baro_pub_ = create_publisher<sensor_msgs::msg::FluidPressure>("/barometer/data", 10);
    battery_pub_ = create_publisher<sensor_msgs::msg::BatteryState>("/ardupilot/battery", 10);
    status_pub_ = create_publisher<std_msgs::msg::String>("/ardupilot/status", 10);

    // -------------------------------------------------------------------------
    // Subscribers (Guidance/Control commands from ROS 2 to ArduPilot)
    // -------------------------------------------------------------------------
    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/ardupilot/cmd_vel", 10,
      std::bind(&ArduPilotNode::cmd_vel_callback, this, std::placeholders::_1));

    cmd_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/ardupilot/cmd_pose", 10,
      std::bind(&ArduPilotNode::cmd_pose_callback, this, std::placeholders::_1));

    // -------------------------------------------------------------------------
    // TODO: MAVLink Connection Setup
    // -------------------------------------------------------------------------
    // TODO:
    //   1. Open serial port or UDP socket based on connection_ parameter:
    //      - e.g. mavlink_status_t, mavlink_message_t
    //   2. Exchange HEARTBEAT with ArduPilot (type MAV_TYPE_QUADROTOR, autopilot MAV_AUTOPILOT_ARDUPILOTMEGA).
    //   3. Request high-rate data streams via MAV_CMD_SET_MESSAGE_INTERVAL:
    //      - MAVLINK_MSG_ID_HIGHRES_IMU / RAW_IMU (e.g. 100 Hz)
    //      - MAVLINK_MSG_ID_GLOBAL_POSITION_INT (e.g. 10 Hz)
    //      - MAVLINK_MSG_ID_SCALED_PRESSURE (e.g. 20 Hz)
    //      - MAVLINK_MSG_ID_SYS_STATUS / BATTERY_STATUS (e.g. 1 Hz)

    RCLCPP_INFO(get_logger(), "ArduPilot Interface skeleton ready (idle mode).");
  }

private:
  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    // -------------------------------------------------------------------------
    // TODO: MAVLink Velocity Setpoint Command
    // -------------------------------------------------------------------------
    // TODO:
    //   1. Encode msg->linear (vx, vy, vz) and msg->angular.z (yaw_rate).
    //   2. Send MAVLink SET_POSITION_TARGET_LOCAL_NED:
    //      - coordinate_frame = MAV_FRAME_BODY_NED or MAV_FRAME_LOCAL_NED
    //      - type_mask = ignore position, ignore accel, enable velocity and yaw_rate.
    (void)msg;
  }

  void cmd_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    // -------------------------------------------------------------------------
    // TODO: MAVLink Position / Waypoint Setpoint Command
    // -------------------------------------------------------------------------
    // TODO:
    //   1. Convert target pose in ENU/NED.
    //   2. Send MAVLink SET_POSITION_TARGET_LOCAL_NED (position target).
    (void)msg;
  }

  // ---------------------------------------------------------------------------
  // TODO: Telemetry Receiver Thread / Polling Loop
  // ---------------------------------------------------------------------------
  // TODO:
  //   In background receiver thread:
  //   - Read MAVLink packet.
  //   - case MAVLINK_MSG_ID_HIGHRES_IMU:
  //       populate sensor_msgs/Imu, publish to imu_pub_.
  //   - case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
  //       populate sensor_msgs/NavSatFix, publish to gps_pub_.
  //   - case MAVLINK_MSG_ID_SCALED_PRESSURE:
  //       populate sensor_msgs/FluidPressure, publish to baro_pub_.

  std::string connection_;
  int baudrate_{921600};
  int system_id_{1};
  int component_id_{1};

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_pub_;
  rclcpp::Publisher<sensor_msgs::msg::FluidPressure>::SharedPtr baro_pub_;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr cmd_pose_sub_;
};

}  // namespace ardupilot_interface

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ardupilot_interface::ArduPilotNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
