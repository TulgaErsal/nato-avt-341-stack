#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include <avt_341_nav/perception_params_dto.hpp>

#include "avt_341_nav/perception/layers/camera_projection_utils.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

namespace
{

image_geometry::PinholeCameraModel MakeCameraModel(const bool distorted)
{
    auto info = std::make_shared<sensor_msgs::msg::CameraInfo>();
    info->width = 640;
    info->height = 480;
    info->distortion_model = "plumb_bob";
    info->d = distorted
        ? std::vector<double>{0.25, -0.05, 0.0, 0.0, 0.0}
        : std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0};
    info->k = {
        300.0, 0.0, 320.0,
        0.0, 300.0, 240.0,
        0.0, 0.0, 1.0};
    info->r = {
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0};
    info->p = {
        300.0, 0.0, 320.0, 0.0,
        0.0, 300.0, 240.0, 0.0,
        0.0, 0.0, 1.0, 0.0};

    image_geometry::PinholeCameraModel model;
    model.fromCameraInfo(info);
    return model;
}

}

TEST(CameraSegPcLayerParameters, DefaultsMatchPublicContract)
{
    const avt_341_nav::params::perception::Params params;
    EXPECT_EQ(params.point_cloud_layer.topic, "avt_341/points");
    EXPECT_EQ(
        params.camera_seg_pc_layer.point_cloud_topic,
        "avt_341/points");
    EXPECT_TRUE(params.camera_seg_pc_layer.segmentation_topic.empty());
    EXPECT_TRUE(params.camera_seg_pc_layer.info_topic.empty());
    EXPECT_EQ(params.camera_seg_pc_layer.use_N_last_scans, 1);
    EXPECT_DOUBLE_EQ(params.camera_seg_pc_layer.pc_move_threshold, 0.25);
}

TEST(CameraProjectionScanSpacing, UsesPlanarDisplacementFromRetainedPose)
{
    using avt_341_nav::perception::camera_projection::MeetsPlanarScanDistance;

    EXPECT_FALSE(MeetsPlanarScanDistance(0.1, 0.1, 0.0, 0.0, 0.25));
    EXPECT_TRUE(MeetsPlanarScanDistance(0.15, 0.2, 0.0, 0.0, 0.25));
    EXPECT_TRUE(MeetsPlanarScanDistance(0.25, 0.0, 0.0, 0.0, 0.25));
    EXPECT_TRUE(MeetsPlanarScanDistance(0.0, 0.0, 0.0, 0.0, 0.0));
}

TEST(CameraProjectionFov, ProjectsOnlyPointsInsideRawImage)
{
    using avt_341_nav::perception::camera_projection::ProjectToRawImage;
    const auto model = MakeCameraModel(false);

    const auto center = ProjectToRawImage(model, cv::Point3d(0.0, 0.0, 2.0), 640, 480);
    ASSERT_TRUE(center.has_value());
    EXPECT_EQ(center->x, 320);
    EXPECT_EQ(center->y, 240);

    EXPECT_FALSE(ProjectToRawImage(model, cv::Point3d(0.0, 0.0, -1.0), 640, 480));
    EXPECT_FALSE(ProjectToRawImage(model, cv::Point3d(3.0, 0.0, 1.0), 640, 480));
    EXPECT_FALSE(ProjectToRawImage(model, cv::Point3d(-3.0, 0.0, 1.0), 640, 480));
    EXPECT_FALSE(ProjectToRawImage(model, cv::Point3d(0.0, 3.0, 1.0), 640, 480));
    EXPECT_FALSE(ProjectToRawImage(model, cv::Point3d(0.0, -3.0, 1.0), 640, 480));
    EXPECT_FALSE(ProjectToRawImage(
        model,
        cv::Point3d(std::numeric_limits<double>::quiet_NaN(), 0.0, 1.0),
        640, 480));
}

TEST(CameraProjectionFov, ConvertsRectifiedProjectionBackToDistortedPixel)
{
    using avt_341_nav::perception::camera_projection::ProjectToRawImage;
    const auto model = MakeCameraModel(true);
    const cv::Point3d point(0.8, 0.2, 2.0);

    const cv::Point2d rectified = model.project3dToPixel(point);
    const cv::Point2d raw = model.unrectifyPoint(rectified);
    const auto pixel = ProjectToRawImage(model, point, 640, 480);

    ASSERT_TRUE(pixel.has_value());
    EXPECT_EQ(pixel->x, static_cast<int>(std::lround(raw.x)));
    EXPECT_EQ(pixel->y, static_cast<int>(std::lround(raw.y)));
    EXPECT_NE(pixel->x, static_cast<int>(std::lround(rectified.x)));
}
