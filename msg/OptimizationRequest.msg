# Optimization Request
std_msgs/Header header 

uint8 robot_id

sensor_msgs/PointCloud2 keyframe

int64 index_to
nav_msgs/Odometry pose_to
int64 index_from
nav_msgs/Odometry pose_from

float32 noise

int32[] other_ids
float32[] distances
float32[] descriptor_vec

int32 gps_valid
nav_msgs/Odometry gps_odom