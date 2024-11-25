# Robot Name
string robotID

# Robot Name Receive
string robotIDReceive

# Cloud Info
Header header 

# SOLiD dimension
int64 numRing
int64 numSector
int64 numHeight

# Present robot pose
float32 poseX
float32 poseY
float32 poseZ
float32 poseRoll
float32 posePitch
float32 poseYaw
float32 poseIntensity

# Point cloud messages
sensor_msgs/PointCloud2 scanCloud  # extracted feature cloud

# SOLiD info
float32[] asolid
float32[] rsolid

# Scan context info
float32[] scanContextBin
float32[] ringKey