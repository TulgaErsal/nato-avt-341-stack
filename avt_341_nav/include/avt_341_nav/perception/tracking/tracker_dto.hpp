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
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
namespace avt_341_nav::perception
{

    /**
         * @brief State of the tracker.
         *
         * - UNINITIALIZED:         The tracker is launching and not yet ready to
                                    track.

         * - INACTIVE:              The tracker has yet to receive an initial valid camera
                                    detection to initialize the tracking.

         * - NO_DETECTION:          No camera or LiDAR detection available: the
                                    estimation filter is publishing a predicted target
                                    odometry.

         * - LIDAR_ONLY_TRACKING:   No camera detection is available: the target
                                    is being tracked on the LiDAR region of
                                    interest through state estimation.

         * - FULL_TRACKING:         Both camera and LiDAR detections are available: the
                                    target is being tracked in the region of interest
                                    defined by the camera.

        * - CAMERA_ONLY_TRACKING:   Only camera detections are available.
                                    Estimates depth based on bounding box relative size.

        * - LOST:                   Tracker is lost. Determined either by no sensory inputs,
                                    high uncertainty, or error in stationary velocity estimate
                                    confirmed by vehicle-to-vehicle communication.
         */
    enum TrackerState {
        UNINITIALIZED = 0,
        INACTIVE = 1,
        NO_DETECTION = 2,
        LIDAR_ONLY_TRACKING = 3,
        FULL_TRACKING = 4,
        CAMERA_ONLY_TRACKING = 5,
        LOST = 6
    };

    /**
     * @brief Concrete type of an ObjectTracker instance.
     *
     * - Generic:          Plain ObjectTracker base class.
     * - Toi:              ToiTracker; target id matches
     *                     "target_selection.toi_regex" and target contacts
     *                     are published.
     * - FormationVehicle: FormationVehicleTracker; target id is one of the
     *                     "target_selection.formation_vehicle_ids" and
     *                     lost-detection/recovery
     *                     is active.
     */
    enum class ObjectTrackerType {
        Generic = 0,
        Toi = 1,
        FormationVehicle = 2
    };

    inline std::string ToString(const ObjectTrackerType type) {
        if (type == ObjectTrackerType::Generic) {
            return "Generic";
        } else if (type == ObjectTrackerType::Toi) {
            return "Toi";
        } else if (type == ObjectTrackerType::FormationVehicle) {
            return "FormationVehicle";
        }
        throw std::invalid_argument("Unknown object tracker type.");
    }

    inline bool IsActiveTrackerState(const TrackerState state) {
        return state == TrackerState::FULL_TRACKING ||
                state == TrackerState::LIDAR_ONLY_TRACKING ||
                state == TrackerState::CAMERA_ONLY_TRACKING;
    }

    /** @brief Inverse of ToString: parse a tracker state name (e.g.
     *         "lidar_only" -> LIDAR_ONLY_TRACKING). Throws on unknown names. */
    inline TrackerState ToTrackerState(const std::string& state) {
        if (state == "uninitialized") {
            return TrackerState::UNINITIALIZED;
        } else if (state == "inactive") {
            return TrackerState::INACTIVE;
        } else if (state == "no_detection") {
            return TrackerState::NO_DETECTION;
        } else if (state == "lidar_only") {
            return TrackerState::LIDAR_ONLY_TRACKING;
        } else if (state == "full") {
            return TrackerState::FULL_TRACKING;
        } else if (state == "camera_only") {
            return TrackerState::CAMERA_ONLY_TRACKING;
        } else if (state == "lost") {
            return TrackerState::LOST;
        }
        throw std::invalid_argument("Unknown tracker state \"" + state + "\".");
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
        } else if (state == TrackerState::LOST) {
            return "lost";
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

		/** @brief Global reference to point-cloud cluster
*        for communication between obstacle detector and detector improvment. */
		pcl::PointCloud<pcl::PointXYZ>::Ptr current_cluster;

    };

    class PixelCoordinates {
    public:
        PixelCoordinates(const int& x, const int& y) : x_(x), y_(y) {}

        int x_;
        int y_;
    };


}

#endif  // AVT_341_TRACKER_DTO_H
