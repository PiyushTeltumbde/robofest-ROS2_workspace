# Status information for 3D mapping and dense volumetric reconstruction
std_msgs/Header header

# Current status of the mapper (e.g., "IDLE", "MAPPING", "SAVING", "OPTIMIZING")
string status

# Total number of 3D points / voxels in the current active map
uint32 total_points

# Total number of integrated keyframes
uint32 keyframes_count

# Flag indicating whether a consistent map is available for path planning
bool map_available

# Voxel grid / octree resolution in meters
float32 resolution
