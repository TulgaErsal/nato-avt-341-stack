# This messages holds information on the tracker status. It is meant to be used
# to retrieve the internal state of the tracker and statistics on the tracking
# process.


# Tracker state enumeration type.
# - UNINITIALIZED: The tracker is launching and not yet ready to track.
# - NO_DETECTION: No camera or LiDAR detection available: the estimation filter
#                 is publishing a predicted target odometry.
# - LIDAR_ONLY_TRACKING: No camera detection is available: the target is being
#                        tracked on the LiDAR region of interest through state
#                        estimation.
# - FULL_TRACKING: Both camera and LiDAR detections are available: the target is
#                  being tracked in the region of interest defined by the
#                  camera.
uint8 STATE_UNINITIALIZED = 0
uint8 STATE_INACTIVE = 1
uint8 STATE_NO_DETECTION = 2
uint8 STATE_LIDAR_ONLY_TRACKING = 3
uint8 STATE_FULL_TRACKING = 4

# Message header stamped on tracker info publishing callback.
std_msgs/Header header

# Current state of the tracker (see enumeration type STATE_).
uint8 state

# Number of points in the tracked object point cloud.
uint32 tracked_cloud_size

# Whether or not the clustering operation was successful.
bool clustering_success

# Number of clusters before tracked object cloud selection.
uint8 clusters_found

# Whether or not the ground segmentation algorithm was successful.
bool ground_segmentation_success

# Number of milliseconds since the last valid camera deteciton.
float32 time_since_valid_detection

# Number of milliseconds required for the tracking pipeline execution.
float32 execution_time
