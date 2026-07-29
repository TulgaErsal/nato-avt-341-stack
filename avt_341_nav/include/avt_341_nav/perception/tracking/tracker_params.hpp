/**
 * @file tracker_params.hpp
 * @brief Thin domain adapters around the generated object-tracking parameters.
 */

#pragma once

#include <string>
#include <vector>

#include <Eigen/Dense>

#include <avt_341_nav/object_tracker_params_dto.hpp>
#include <avt_341_nav/perception/tracking/tracker_dto.hpp>

namespace avt_341_nav::perception {

using ObjectTrackerSettings = avt_341_nav::params::object_tracker::Params;
using RecoverySettings = ObjectTrackerSettings::Recovery;

/** Resolve a frame id using the configured tracker frame prefix. */
std::string ResolveFrameId(const ObjectTrackerSettings::Frames& frames,
                           const std::string& frame_id);

/** Resolve the configured camera frame id. */
std::string ResolveCameraFrame(const ObjectTrackerSettings& params);

/** Resolve the configured obstacle-detector base frame id. */
std::string ResolveRobotBaseLink(const ObjectTrackerSettings& params);

/** Convert a validated xyz parameter array to an Eigen vector. */
Eigen::Vector3f ToEigenVector3f(const std::vector<double>& values);

/** Convert a validated xyz parameter array to a homogeneous Eigen point. */
Eigen::Vector4f ToEigenPoint4f(const std::vector<double>& values);

/** Test whether a tracker state is present in the configured string list. */
bool IsConfiguredTrackerState(const RecoverySettings& params,
                              TrackerState state);

/**
 * Apply the runtime-reconfigurable subset supported by the original tracker.
 *
 * The generated listener owns validation and supplies a complete updated
 * snapshot. This adapter deliberately copies only the settings that the
 * tracker historically applied without reconstructing ROS resources.
 */
bool ApplyRuntimeParameters(ObjectTrackerSettings& params,
                            const ObjectTrackerSettings& updated_params);

}  // namespace avt_341_nav::perception
