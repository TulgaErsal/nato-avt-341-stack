#include <gtest/gtest.h>

#include <stdexcept>
#include <vector>

#include <avt_341/perception/tracking/tracker_params.hpp>

namespace {

using avt_341::perception::ApplyRuntimeParameters;
using avt_341::perception::IsConfiguredTrackerState;
using avt_341::perception::ObjectTrackerSettings;
using avt_341::perception::ResolveCameraFrame;
using avt_341::perception::ResolveRobotBaseLink;
using avt_341::perception::ToEigenPoint4f;
using avt_341::perception::ToEigenVector3f;
using avt_341::perception::ToTrackerState;
using avt_341::perception::TrackerState;

TEST(TrackerParams, ResolvesPrefixedFrames) {
    ObjectTrackerSettings params;
    params.frames.prefix = "vehicle/";
    params.frames.camera_frame = "camera";
    params.obstacle_detector.robot_base_link = "base_link";

    EXPECT_EQ(ResolveCameraFrame(params), "vehicle/camera");
    EXPECT_EQ(ResolveRobotBaseLink(params), "vehicle/base_link");
}

TEST(TrackerParams, ConvertsValidatedFixedSizeArrays) {
    const std::vector<double> xyz{1.0, 2.0, 3.0};

    const auto vector = ToEigenVector3f(xyz);
    EXPECT_FLOAT_EQ(vector.x(), 1.0f);
    EXPECT_FLOAT_EQ(vector.y(), 2.0f);
    EXPECT_FLOAT_EQ(vector.z(), 3.0f);

    const auto point = ToEigenPoint4f(xyz);
    EXPECT_FLOAT_EQ(point.x(), 1.0f);
    EXPECT_FLOAT_EQ(point.y(), 2.0f);
    EXPECT_FLOAT_EQ(point.z(), 3.0f);
    EXPECT_FLOAT_EQ(point.w(), 1.0f);
}

TEST(TrackerParams, ComparesAndParsesStringStates) {
    ObjectTrackerSettings::Recovery recovery;
    recovery.no_movement_check_in_states = {"lidar_only", "full"};

    EXPECT_TRUE(IsConfiguredTrackerState(
        recovery, TrackerState::LIDAR_ONLY_TRACKING));
    EXPECT_TRUE(IsConfiguredTrackerState(
        recovery, TrackerState::FULL_TRACKING));
    EXPECT_FALSE(IsConfiguredTrackerState(
        recovery, TrackerState::CAMERA_ONLY_TRACKING));
    EXPECT_EQ(ToTrackerState("camera_only"),
              TrackerState::CAMERA_ONLY_TRACKING);
    EXPECT_THROW(ToTrackerState("not_a_tracker_state"),
                 std::invalid_argument);
}

TEST(TrackerParams, AppliesOnlySupportedRuntimeSubset) {
    ObjectTrackerSettings params;
    ObjectTrackerSettings updated = params;
    updated.tracking.target_timeout = 8.0;
    updated.sync.max_detection_skew = 0.25;
    updated.target_selection.formation_vehicle_ids = {"agv1"};

    EXPECT_TRUE(ApplyRuntimeParameters(params, updated));
    EXPECT_DOUBLE_EQ(params.tracking.target_timeout, 8.0);
    EXPECT_DOUBLE_EQ(params.sync.max_detection_skew, 0.25);
    EXPECT_TRUE(params.target_selection.formation_vehicle_ids.empty());
    EXPECT_FALSE(ApplyRuntimeParameters(params, params));
}

}  // namespace
