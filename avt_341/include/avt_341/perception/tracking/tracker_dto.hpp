/**
* @file      tracker_dto.hpp
* @brief     Data transfer objects shared between the object tracking node
             and the replicated per-target ObjectTracker instances.
*/

#ifndef AVT_341_TRACKER_DTO_H
#define AVT_341_TRACKER_DTO_H

#include <stdexcept>
#include <string>

#include <sensor_msgs/msg/camera_info.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace avt_341::perception
{

    /**
         * @brief State of the tracker.
         *
         * - UNININITIALIZED: The tracker is launching and not yet ready to
                              track.
         * - INACTIVE: The tracker has yet to receive an initial valid camera
                       detection to initialize the tracking.
         * - NO_DETECTION: No camera or LiDAR detection available: the
                           estimation filter is publishing a predicted target
                           odometry.
         * - LIDAR_ONLY_TRACKING: No camera detection is available: the target
                                  is being tracked on the LiDAR region of
                                  interest through state estimation.
         * - FULL_TRACKING: Both camera and LiDAR detections are available: the
                            target is being tracked in the region of interest
                            defined by the camera.
         */
    enum TrackerState {
        UNINITIALIZED = 0,
        INACTIVE = 1,
        NO_DETECTION = 2,
        LIDAR_ONLY_TRACKING = 3,
        FULL_TRACKING = 4,
        CAMERA_ONLY_TRACKING = 5
    };

    inline bool IsActiveTrackerState(const TrackerState state) {
        return state == TrackerState::FULL_TRACKING ||
                state == TrackerState::LIDAR_ONLY_TRACKING ||
                state == TrackerState::CAMERA_ONLY_TRACKING;
    }

    inline std::string ToString(const TrackerState state) {
        if (state == TrackerState::UNINITIALIZED) {
            return "uninitialized";
        } else if (state == TrackerState::INACTIVE) {
            return "inactive";
        } else if (state == TrackerState::NO_DETECTION) {
            return "no_detection";
        } else if (state == TrackerState::LIDAR_ONLY_TRACKING) {
            return "lidar_only";
        } else if (state == TrackerState::FULL_TRACKING) {
            return "full";
        } else if (state == TrackerState::CAMERA_ONLY_TRACKING) {
            return "camera_only";
        }
        throw std::invalid_argument("Unknown tracker state.");
    }

    /** @brief Shared per-tick sensor context owned by the node and passed to
 *         every ObjectTracker instance on each tracking tick. */
    struct TrackerSensorContext {
        /** @brief Latest MarkerArray from the node's integrated obstacle
     *         detector. */
        const visualization_msgs::msg::MarkerArray* obstacle_markers = nullptr;

        /** @brief True once at least one MarkerArray (including DELETEALL) has
     *         been produced by the obstacle detector. */
        bool has_obstacle_markers = false;

        /** @brief Latest camera intrinsics; nullptr until the first CameraInfo
     *         message has been received. */
        sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info;
    };

    class PixelCoordinates {
    public:
        PixelCoordinates(const int& x, const int& y) : x_(x), y_(y) {}

        int x_;
        int y_;
    };


}

#endif  // AVT_341_TRACKER_DTO_H
