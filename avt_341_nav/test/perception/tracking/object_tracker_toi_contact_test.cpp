// Unit tests for where the object tracking node places a target of interest.
//
// A camera-only TOI detection must produce target contacts at the detected
// target, never at the camera itself (the filter used to be seeded at the
// camera origin before the first measurement), and the camera intrinsics must
// follow the binning and ROI of a downsized image's CameraInfo.

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <vision_msgs/msg/detection2_d_array.hpp>

#include <avt_341_nav/perception/tracking/object_tracker_node.hpp>

#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <vector>

using namespace std::chrono_literals;

namespace {

const std::string TARGET = "TI_car";
const std::string CAMERA_FRAME = "test_camera";

// Camera at (100, 200, 1) in map, looking along +x (optical frame).
constexpr double CAMERA_X = 100.0;
constexpr double CAMERA_Y = 200.0;
constexpr double CAMERA_Z = 1.0;
constexpr double TARGET_RANGE = 20.0;

// Full-resolution calibration of a 2448x2048 camera, published for a
// 608x512 image cropped to 2432 columns at x offset 8 and decimated by 4.
constexpr double FULL_F = 1600.0;
constexpr double FULL_CX = 1232.0;
constexpr double FULL_CY = 1024.0;
constexpr double IMAGE_F = FULL_F / 4.0;
constexpr double IMAGE_CX = (FULL_CX - 8.0) / 4.0;
constexpr double IMAGE_CY = FULL_CY / 4.0;

}  // namespace

class ObjectTrackerToiContactTest : public ::testing::Test {
protected:
    void SetUp() override {
        tracking_node_ = std::make_shared<avt_341_nav::perception::ObjectTrackerNode>();
        helper_node_ = rclcpp::Node::make_shared("toi_contact_test_helper");

        tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(helper_node_);
        geometry_msgs::msg::TransformStamped camera_tf;
        camera_tf.header.frame_id = "map";
        camera_tf.child_frame_id = CAMERA_FRAME;
        camera_tf.transform.translation.x = CAMERA_X;
        camera_tf.transform.translation.y = CAMERA_Y;
        camera_tf.transform.translation.z = CAMERA_Z;
        camera_tf.transform.rotation.x = -0.5;
        camera_tf.transform.rotation.y = 0.5;
        camera_tf.transform.rotation.z = -0.5;
        camera_tf.transform.rotation.w = 0.5;
        tf_broadcaster_->sendTransform(camera_tf);

        camera_info_pub_ = helper_node_->create_publisher<sensor_msgs::msg::CameraInfo>("camera_info", 10);
        detections_pub_ = helper_node_->create_publisher<vision_msgs::msg::Detection2DArray>("detection_2d", 10);
        contacts_sub_ = helper_node_->create_subscription<nav_msgs::msg::Path>(
            "avt_341/target_contacts", 10,
            [this](nav_msgs::msg::Path::SharedPtr msg) { contacts_.push_back(*msg); });
        estimate_sub_ = helper_node_->create_subscription<nav_msgs::msg::Odometry>(
            "avt_341/odometry/estimated/ti_car", 10,
            [this](nav_msgs::msg::Odometry::SharedPtr msg) { estimates_.push_back(*msg); });

        exec_.add_node(tracking_node_);
        exec_.add_node(helper_node_);
        Spin(300ms);
    }

    void TearDown() override {
        exec_.remove_node(tracking_node_);
        exec_.remove_node(helper_node_);
    }

    void SpinUntil(std::function<bool()> condition, std::chrono::milliseconds timeout) {
        auto deadline = std::chrono::steady_clock::now() + timeout;
        while (!condition() && std::chrono::steady_clock::now() < deadline) {
            exec_.spin_some(10ms);
        }
    }

    void Spin(std::chrono::milliseconds duration) {
        SpinUntil([] { return false; }, duration);
    }

    void PublishCameraInfo() {
        sensor_msgs::msg::CameraInfo info;
        info.header.frame_id = CAMERA_FRAME;
        info.width = 2448;
        info.height = 2048;
        info.k = {FULL_F, 0.0, FULL_CX, 0.0, FULL_F, FULL_CY, 0.0, 0.0, 1.0};
        info.p = {FULL_F, 0.0, FULL_CX, 0.0, 0.0, FULL_F, FULL_CY, 0.0, 0.0, 0.0, 1.0, 0.0};
        info.binning_x = 4;
        info.binning_y = 4;
        info.roi.x_offset = 8;
        info.roi.width = 2432;
        info.roi.height = 2048;
        camera_info_pub_->publish(info);
    }

    // A detection centered on the optical axis whose height puts the target
    // TARGET_RANGE ahead of the camera in the 608x512 image.
    void PublishDetection() {
        vision_msgs::msg::Detection2DArray detections;
        detections.header.stamp = tracking_node_->get_clock()->now();
        detections.header.frame_id = CAMERA_FRAME;
        vision_msgs::msg::Detection2D detection;
        detection.bbox.center.position.x = IMAGE_CX;
        detection.bbox.center.position.y = IMAGE_CY;
        detection.bbox.size_y = IMAGE_F * 1.9 / TARGET_RANGE;
        detection.bbox.size_x = 1.5 * detection.bbox.size_y;
        vision_msgs::msg::ObjectHypothesisWithPose hypothesis;
        hypothesis.hypothesis.class_id = TARGET;
        hypothesis.hypothesis.score = 0.9;
        detection.results.push_back(hypothesis);
        detections.detections.push_back(detection);
        detections_pub_->publish(detections);
    }

    std::shared_ptr<avt_341_nav::perception::ObjectTrackerNode> tracking_node_;
    std::shared_ptr<rclcpp::Node> helper_node_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
    rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr detections_pub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr contacts_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr estimate_sub_;
    rclcpp::executors::SingleThreadedExecutor exec_;
    std::vector<nav_msgs::msg::Path> contacts_;
    std::vector<nav_msgs::msg::Odometry> estimates_;
};

TEST_F(ObjectTrackerToiContactTest, CameraOnlyContactsAreAtTheTargetNotTheCamera) {
    PublishCameraInfo();
    Spin(100ms);
    for (int i = 0; i < 20; ++i) {
        PublishDetection();
        Spin(100ms);
    }

    ASSERT_FALSE(contacts_.empty());
    for (const auto& contact : contacts_) {
        ASSERT_EQ(contact.poses.size(), 1u);
        const auto& p = contact.poses.front().pose.position;
        EXPECT_NEAR(p.x, CAMERA_X + TARGET_RANGE, 0.5);
        EXPECT_NEAR(p.y, CAMERA_Y, 0.5);
        EXPECT_NEAR(p.z, CAMERA_Z, 0.5);
    }
    ASSERT_FALSE(estimates_.empty());
    for (const auto& estimate : estimates_) {
        const auto& p = estimate.pose.pose.position;
        EXPECT_GT(std::hypot(p.x - CAMERA_X, p.y - CAMERA_Y), 0.5 * TARGET_RANGE);
    }
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    std::vector<const char*> args(argv, argv + argc);
    args.insert(args.end(), {"--ros-args", "-p", "frames.use_vehicle_prefix:=false",
                             "-p", "frames.camera:=test_camera"});
    rclcpp::init(static_cast<int>(args.size()), args.data());
    int result = RUN_ALL_TESTS();
    rclcpp::shutdown();
    return result;
}
