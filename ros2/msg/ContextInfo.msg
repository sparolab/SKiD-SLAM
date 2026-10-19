# Robot Name
string robot_id

# Robot Name Receive
string robot_id_receive

# Cloud Info
std_msgs/Header header

# SOLiD dimension
int64 num_ring
int64 num_sector
int64 num_height

# Present robot pose
float32 pose_x
float32 pose_y
float32 pose_z
float32 pose_roll
float32 pose_pitch
float32 pose_yaw
float32 pose_intensity

# Point cloud messages
sensor_msgs/PointCloud2 scan_cloud  # extracted feature cloud

# SOLiD info
float32[] asolid
float32[] rsolid
