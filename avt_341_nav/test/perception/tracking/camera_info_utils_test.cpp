// Unit tests for ToImageCameraInfo: the intrinsics of the image a CameraInfo
// describes, with binning and ROI applied.

#include <gtest/gtest.h>

#include <avt_341_nav/perception/tracking/camera_info_utils.hpp>

using avt_341_nav::perception::ToImageCameraInfo;

namespace {

// The UM MRZR FLIR calibration as published on /flir_camera_downsized/camera_info
// by image_proc CropDecimate (offset_x 8, width 2432, decimation 4).
sensor_msgs::msg::CameraInfo MakeDownsizedFlirInfo() {
    sensor_msgs::msg::CameraInfo info;
    info.width = 2448;
    info.height = 2048;
    info.k = {1454.7885, 0.0, 1212.21805, 0.0, 1452.16383, 1021.03138, 0.0, 0.0, 1.0};
    info.p = {1492.93518, 0.0, 1212.78488, 0.0, 0.0, 1490.43152, 1018.64775, 0.0, 0.0, 0.0, 1.0, 0.0};
    info.binning_x = 4;
    info.binning_y = 4;
    info.roi.x_offset = 8;
    info.roi.width = 2432;
    info.roi.height = 2048;
    info.roi.do_rectify = true;
    return info;
}

}  // namespace

TEST(ToImageCameraInfo, DownsizedImageGetsBinnedAndCroppedIntrinsics) {
    const auto out = ToImageCameraInfo(MakeDownsizedFlirInfo());
    EXPECT_EQ(out.width, 608u);
    EXPECT_EQ(out.height, 512u);
    EXPECT_NEAR(out.k[0], 1492.93518 / 4.0, 1e-9);
    EXPECT_NEAR(out.k[2], (1212.78488 - 8.0) / 4.0, 1e-9);
    EXPECT_NEAR(out.k[4], 1490.43152 / 4.0, 1e-9);
    EXPECT_NEAR(out.k[5], 1018.64775 / 4.0, 1e-9);
    EXPECT_DOUBLE_EQ(out.p[0], out.k[0]);
    EXPECT_DOUBLE_EQ(out.p[2], out.k[2]);
    EXPECT_DOUBLE_EQ(out.p[5], out.k[4]);
    EXPECT_DOUBLE_EQ(out.p[6], out.k[5]);
    EXPECT_EQ(out.binning_x, 0u);
    EXPECT_EQ(out.binning_y, 0u);
    EXPECT_EQ(out.roi.width, 0u);
    EXPECT_EQ(out.roi.x_offset, 0u);
}

TEST(ToImageCameraInfo, FullResolutionInfoKeepsItsSize) {
    auto info = MakeDownsizedFlirInfo();
    info.binning_x = 0;
    info.binning_y = 0;
    info.roi = sensor_msgs::msg::RegionOfInterest();
    const auto out = ToImageCameraInfo(info);
    EXPECT_EQ(out.width, 2448u);
    EXPECT_EQ(out.height, 2048u);
    EXPECT_DOUBLE_EQ(out.k[0], 1492.93518);
    EXPECT_DOUBLE_EQ(out.k[2], 1212.78488);
}

TEST(ToImageCameraInfo, MissingProjectionFallsBackToK) {
    auto info = MakeDownsizedFlirInfo();
    info.p = {};
    const auto out = ToImageCameraInfo(info);
    EXPECT_NEAR(out.k[0], 1454.7885 / 4.0, 1e-9);
    EXPECT_NEAR(out.k[2], (1212.21805 - 8.0) / 4.0, 1e-9);
    EXPECT_NEAR(out.k[5], 1021.03138 / 4.0, 1e-9);
}

TEST(ToImageCameraInfo, EmptyInfoStaysEmpty) {
    sensor_msgs::msg::CameraInfo info;
    info.width = 640;
    info.height = 480;
    const auto out = ToImageCameraInfo(info);
    EXPECT_EQ(out.width, 640u);
    EXPECT_EQ(out.height, 480u);
    EXPECT_DOUBLE_EQ(out.k[0], 0.0);
}
