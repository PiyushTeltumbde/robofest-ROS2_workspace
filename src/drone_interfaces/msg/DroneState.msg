# Comprehensive state representation for the autonomous quadcopter
std_msgs/Header header

# 3D Position in world/odom frame (meters)
geometry_msgs/Point position

# Linear velocity in body or world frame (m/s)
geometry_msgs/Vector3 velocity

# Orientation in quaternion (w, x, y, z)
geometry_msgs/Quaternion orientation

# SLAM / Odometry tracking status (e.g., "TRACKING", "LOST", "INITIALIZING")
string tracking_status

# ArduPilot vehicle armed status
bool armed

# ArduPilot flight mode (e.g., "GUIDED", "LOITER", "AUTO", "RTL")
string flight_mode

# Battery state of charge (percentage 0.0 - 100.0)
float32 battery_percentage
