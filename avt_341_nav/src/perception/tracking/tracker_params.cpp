/**
 * @file tracker_params.cpp
 * @brief Domain adapters for generated object-tracking parameters.
 */

#include <avt_341/perception/tracking/tracker_params.hpp>

#include <algorithm>

namespace avt_341::perception {

std::string ResolveFrameId(const ObjectTrackerSettings::Frames& frames,
                           const std::string& frame_id) {
    return frames.prefix + frame_id;
}

std::string ResolveCameraFrame(const ObjectTrackerSettings& params) {
    return ResolveFrameId(params.frames, params.frames.camera_frame);
}

std::string ResolveRobotBaseLink(const ObjectTrackerSettings& params) {
    return ResolveFrameId(params.frames,
                          params.obstacle_detector.robot_base_link);
}

Eigen::Vector3f ToEigenVector3f(const std::vector<double>& values) {
    return Eigen::Vector3f(static_cast<float>(values.at(0)),
                           static_cast<float>(values.at(1)),
                           static_cast<float>(values.at(2)));
}

Eigen::Vector4f ToEigenPoint4f(const std::vector<double>& values) {
    return Eigen::Vector4f(static_cast<float>(values.at(0)),
                           static_cast<float>(values.at(1)),
                           static_cast<float>(values.at(2)), 1.0f);
}

bool IsConfiguredTrackerState(const RecoverySettings& params,
                              const TrackerState state) {
    const std::string state_name = ToString(state);
    return std::find(params.no_movement_check_in_states.begin(),
                     params.no_movement_check_in_states.end(),
                     state_name) != params.no_movement_check_in_states.end();
}

bool ApplyRuntimeParameters(ObjectTrackerSettings& params,
                            const ObjectTrackerSettings& updated_params) {
    bool changed = false;
    const auto assign_if_changed = [&changed](auto& destination,
                                              const auto& source) {
        if (destination != source) {
            destination = source;
            changed = true;
        }
    };

    assign_if_changed(params.frames.camera_frame,
                      updated_params.frames.camera_frame);
    assign_if_changed(params.frames.world_frame,
                      updated_params.frames.world_frame);
    assign_if_changed(params.filter.roi_scale_factor,
                      updated_params.filter.roi_scale_factor);
    assign_if_changed(params.filter.estimator_rate,
                      updated_params.filter.estimator_rate);
    assign_if_changed(params.filter.process_variance,
                      updated_params.filter.process_variance);
    assign_if_changed(params.filter.measurement_variance,
                      updated_params.filter.measurement_variance);
    assign_if_changed(params.tracking.target_timeout,
                      updated_params.tracking.target_timeout);
    assign_if_changed(params.sync.enabled, updated_params.sync.enabled);
    assign_if_changed(params.sync.use_callback_time,
                      updated_params.sync.use_callback_time);
    assign_if_changed(params.sync.max_detection_skew,
                      updated_params.sync.max_detection_skew);
    assign_if_changed(params.tracking.heading_min_speed,
                      updated_params.tracking.heading_min_speed);
    assign_if_changed(params.tracking.heading_resume_speed,
                      updated_params.tracking.heading_resume_speed);
    assign_if_changed(params.filter.use_manual_roi,
                      updated_params.filter.use_manual_roi);
    assign_if_changed(params.target_selection.toi_regex,
                      updated_params.target_selection.toi_regex);
    return changed;
}

}  // namespace avt_341::perception
