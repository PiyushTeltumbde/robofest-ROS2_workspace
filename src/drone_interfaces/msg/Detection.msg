# Object / Survivor detection result for search-and-rescue
std_msgs/Header header

# Class label (e.g., "person", "survivor", "survivor_beacon", "vehicle")
string class_name

# Detection confidence score [0.0 - 1.0]
float32 confidence

# Estimated 3D world position of the detected target (meters)
geometry_msgs/Point position

# 2D image bounding box: [x_min, y_min, x_max, y_max] in normalized or pixel coordinates
float32[4] bbox
