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
#include <avt_341_msgs/msg/target_reports.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

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
        PublishCameraTf(-0.5, 0.5, -0.5, 0.5);

        camera_info_pub_ = helper_node_->create_publisher<sensor_msgs::msg::CameraInfo>("camera_info", 10);
        detections_pub_ = helper_node_->create_publisher<vision_msgs::msg::Detection2DArray>("detection_2d", 10);
        bboxes_pub_ = helper_node_->create_publisher<visualization_msgs::msg::MarkerArray>("obstacles/bboxes", 10);
        clusters_pub_ = helper_node_->create_publisher<sensor_msgs::msg::PointCloud2>("obstacles/cloud_clusters", 10);
        investigated_pub_ = helper_node_->create_publisher<nav_msgs::msg::Path>(
            "avt_341/investigated_contacts", rclcpp::QoS(1).transient_local());
        contacts_sub_ = helper_node_->create_subscription<avt_341_msgs::msg::TargetReports>(
            "avt_341/target_reports", 10,
            [this](avt_341_msgs::msg::TargetReports::SharedPtr msg) {
                if (msg->reports.empty()) {
                    return;
                }
                // Sightings (reports without a track, track_start 0) are kept apart from track reports.
                (rclcpp::Time(msg->reports.front().track_start).nanoseconds() == 0 ? sightings_ : contacts_).push_back(*msg);
            });
        released_sub_ = helper_node_->create_subscription<avt_341_msgs::msg::TargetReports>(
            "avt_341/target_released", 10,
            [this](avt_341_msgs::msg::TargetReports::SharedPtr msg) { released_.push_back(*msg); });
        estimate_sub_ = helper_node_->create_subscription<nav_msgs::msg::Odometry>(
            "avt_341/odometry/estimated/ti_car", 10,
            [this](nav_msgs::msg::Odometry::SharedPtr msg) { estimates_.push_back(*msg); });

        exec_.add_node(tracking_node_);
        exec_.add_node(helper_node_);
        SetReleaseTime(0.5);
        Spin(300ms);
    }

    void SetReleaseTime(double seconds) {
        tracking_node_->set_parameter(
            rclcpp::Parameter("target_selection.investigated_toi_release_time", seconds));
    }

    void TearDown() override {
        exec_.remove_node(tracking_node_);
        exec_.remove_node(helper_node_);
    }

    // Camera at (CAMERA_X, CAMERA_Y, CAMERA_Z) in map; (-0.5, 0.5, -0.5, 0.5)
    // looks along +x.
    void PublishCameraTf(double qx, double qy, double qz, double qw) {
        geometry_msgs::msg::TransformStamped camera_tf;
        camera_tf.header.frame_id = "map";
        camera_tf.child_frame_id = CAMERA_FRAME;
        camera_tf.transform.translation.x = CAMERA_X;
        camera_tf.transform.translation.y = CAMERA_Y;
        camera_tf.transform.translation.z = CAMERA_Z;
        camera_tf.transform.rotation.x = qx;
        camera_tf.transform.rotation.y = qy;
        camera_tf.transform.rotation.z = qz;
        camera_tf.transform.rotation.w = qw;
        tf_broadcaster_->sendTransform(camera_tf);
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
        PublishDetections({0.0});
    }

    // One detection per target TARGET_RANGE ahead of the camera and offset
    // by the given map y [m], in the given order. A detection's box height can
    // instead suggest the given apparent range.
    void PublishDetections(const std::vector<double>& y_offsets,
                           const std::vector<double>& apparent_ranges = {}) {
        vision_msgs::msg::Detection2DArray detections;
        detections.header.stamp = tracking_node_->get_clock()->now();
        detections.header.frame_id = CAMERA_FRAME;
        for (size_t i = 0; i < y_offsets.size(); ++i) {
            const double apparent_range = i < apparent_ranges.size() ? apparent_ranges[i] : TARGET_RANGE;
            vision_msgs::msg::Detection2D detection;
            detection.bbox.center.position.x = IMAGE_CX - IMAGE_F * y_offsets[i] / apparent_range;
            detection.bbox.center.position.y = IMAGE_CY;
            detection.bbox.size_y = IMAGE_F * 1.9 / apparent_range;
            detection.bbox.size_x = 1.5 * detection.bbox.size_y;
            vision_msgs::msg::ObjectHypothesisWithPose hypothesis;
            hypothesis.hypothesis.class_id = TARGET;
            hypothesis.hypothesis.score = 0.9;
            detection.results.push_back(hypothesis);
            detections.detections.push_back(detection);
        }
        detections_pub_->publish(detections);
    }

    void PublishInvestigated(double x, double y) {
        nav_msgs::msg::Path investigated;
        investigated.header.frame_id = "map";
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = TARGET;
        pose.pose.position.x = x;
        pose.pose.position.y = y;
        investigated.poses.push_back(pose);
        investigated_pub_->publish(investigated);
    }

    struct Obstacle {
        int id;
        double x;
        double y;
        double length = 4.5;
        double height = 1.5;
    };

    // LiDAR obstacle boxes (car-sized) at map positions, with their cluster.
    void PublishObstacles(const std::vector<Obstacle>& obstacles) {
        std_msgs::msg::Header header;
        header.stamp = helper_node_->get_clock()->now();
        header.frame_id = "map";

        sensor_msgs::msg::PointCloud2 cloud;
        cloud.header = header;
        sensor_msgs::PointCloud2Modifier modifier(cloud);
        modifier.setPointCloud2FieldsByString(1, "xyz");
        modifier.resize(4 * obstacles.size());
        sensor_msgs::PointCloud2Iterator<float> it_x(cloud, "x"), it_y(cloud, "y"), it_z(cloud, "z");
        visualization_msgs::msg::MarkerArray markers;
        for (const auto& obstacle : obstacles) {
            for (int i = 0; i < 4; ++i, ++it_x, ++it_y, ++it_z) {
                *it_x = static_cast<float>(obstacle.x + (i % 2 ? 1.0 : -1.0));
                *it_y = static_cast<float>(obstacle.y + (i / 2 ? 0.5 : -0.5));
                *it_z = static_cast<float>(CAMERA_Z);
            }
            visualization_msgs::msg::Marker box;
            box.header = header;
            box.ns = "lidar_bboxes";
            box.id = obstacle.id;
            box.type = visualization_msgs::msg::Marker::CUBE;
            box.action = visualization_msgs::msg::Marker::ADD;
            box.pose.position.x = obstacle.x;
            box.pose.position.y = obstacle.y;
            box.pose.position.z = CAMERA_Z;
            box.pose.orientation.w = 1.0;
            box.scale.x = obstacle.length;
            box.scale.y = 1.8;
            box.scale.z = obstacle.height;
            markers.markers.push_back(box);
        }
        clusters_pub_->publish(cloud);
        bboxes_pub_->publish(markers);
    }

    void PublishObstacle(double x, double y) {
        PublishObstacles({{7, x, y}});
    }

    // Each frame: the given detections and, if requested, a LiDAR box at
    // obstacle_xy.
    void TrackFrames(const std::vector<double>& y_offsets, int frames,
                     const std::vector<double>& apparent_ranges = {},
                     const std::vector<double>& obstacle_xy = {}) {
        for (int i = 0; i < frames; ++i) {
            if (obstacle_xy.size() == 2) {
                PublishObstacle(obstacle_xy[0], obstacle_xy[1]);
            }
            PublishDetections(y_offsets, apparent_ranges);
            Spin(100ms);
        }
    }

    std::shared_ptr<avt_341_nav::perception::ObjectTrackerNode> tracking_node_;
    std::shared_ptr<rclcpp::Node> helper_node_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
    rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr detections_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr investigated_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr bboxes_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr clusters_pub_;
    rclcpp::Subscription<avt_341_msgs::msg::TargetReports>::SharedPtr contacts_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr estimate_sub_;
    rclcpp::Subscription<avt_341_msgs::msg::TargetReports>::SharedPtr released_sub_;
    std::vector<avt_341_msgs::msg::TargetReports> released_;
    rclcpp::executors::SingleThreadedExecutor exec_;
    std::vector<avt_341_msgs::msg::TargetReports> contacts_;
    std::vector<avt_341_msgs::msg::TargetReports> sightings_;
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
        ASSERT_EQ(contact.reports.size(), 1u);
        const auto& p = contact.reports.front().position;
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

// A second target of the same class to the right of the tracked one: near
// enough to pass the filter's gate, and far enough to be investigated on its own.
constexpr double NEAR_Y_OFFSET = -4.0;
constexpr double OTHER_Y_OFFSET = -10.0;

TEST_F(ObjectTrackerToiContactTest, TrackedTargetKeepsItsDetectionWhenAnotherIsListedFirst) {
    PublishCameraInfo();
    Spin(100ms);
    TrackFrames({0.0}, 20);
    ASSERT_FALSE(contacts_.empty());

    contacts_.clear();
    TrackFrames({NEAR_Y_OFFSET, 0.0}, 50);

    ASSERT_FALSE(contacts_.empty());
    for (const auto& contact : contacts_) {
        const auto& p = contact.reports.front().position;
        EXPECT_NEAR(p.x, CAMERA_X + TARGET_RANGE, 0.5);
        EXPECT_NEAR(p.y, CAMERA_Y, 0.5);
    }
}

TEST_F(ObjectTrackerToiContactTest, InvestigatedTargetIsReleasedForTheNextOne) {
    PublishCameraInfo();
    Spin(100ms);
    TrackFrames({0.0, OTHER_Y_OFFSET}, 20);
    ASSERT_FALSE(contacts_.empty());
    EXPECT_NEAR(contacts_.back().reports.front().position.y, CAMERA_Y, 0.5);

    PublishInvestigated(CAMERA_X + TARGET_RANGE, CAMERA_Y);
    Spin(800ms);
    contacts_.clear();
    TrackFrames({0.0, OTHER_Y_OFFSET}, 30);

    ASSERT_FALSE(contacts_.empty());
    for (const auto& contact : contacts_) {
        const auto& p = contact.reports.front().position;
        EXPECT_NEAR(p.x, CAMERA_X + TARGET_RANGE, 0.5);
        EXPECT_NEAR(p.y, CAMERA_Y + OTHER_Y_OFFSET, 0.5);
    }
}

TEST_F(ObjectTrackerToiContactTest, TrackReleasedAtAnInvestigatedTargetReportsItsLastPosition) {
    PublishCameraInfo();
    Spin(100ms);
    TrackFrames({0.0}, 20);
    const size_t published = contacts_.size();
    ASSERT_GT(published, 0u);
    for (int i = 0; i < 20 && contacts_.size() == published; ++i) {
        TrackFrames({0.0}, 1);
    }
    ASSERT_GT(contacts_.size(), published);
    const int32_t track_sec = contacts_.back().reports.front().track_start.sec;
    const uint32_t track_nanosec = contacts_.back().reports.front().track_start.nanosec;
    contacts_.clear();

    PublishInvestigated(CAMERA_X + TARGET_RANGE + 2.0, CAMERA_Y);
    TrackFrames({0.0}, 8);

    ASSERT_EQ(contacts_.size(), 1u);
    const auto& last = contacts_.front().reports.front();
    EXPECT_EQ(last.track_start.sec, track_sec);
    EXPECT_EQ(last.track_start.nanosec, track_nanosec);
    EXPECT_NEAR(last.position.x, CAMERA_X + TARGET_RANGE, 1.0);
    EXPECT_NEAR(last.position.y, CAMERA_Y, 1.0);
    ASSERT_EQ(released_.size(), 1u);
    EXPECT_EQ(released_.front().reports.front().track_start.sec, track_sec);
    EXPECT_EQ(released_.front().reports.front().track_start.nanosec, track_nanosec);
    EXPECT_NEAR(released_.front().reports.front().position.x, last.position.x, 1e-6);
}

TEST_F(ObjectTrackerToiContactTest, TrackOnAnInvestigatedTargetIsReleasedOnlyAfterTheHoldTime) {
    SetReleaseTime(0.5);
    PublishCameraInfo();
    Spin(100ms);
    TrackFrames({0.0}, 20);
    ASSERT_FALSE(contacts_.empty());

    PublishInvestigated(CAMERA_X + TARGET_RANGE + 2.0, CAMERA_Y);
    TrackFrames({0.0}, 3);
    EXPECT_TRUE(released_.empty());
    TrackFrames({0.0}, 5);
    EXPECT_EQ(released_.size(), 1u);
}

TEST_F(ObjectTrackerToiContactTest, SingleFramesWithARangeErrorOntoAnInvestigatedTargetDoNotReleaseTheTrack) {
    SetReleaseTime(0.5);
    PublishCameraInfo();
    Spin(100ms);
    TrackFrames({0.0}, 20);
    ASSERT_FALSE(contacts_.empty());
    const auto track = rclcpp::Time(contacts_.back().reports.front().track_start);

    PublishInvestigated(CAMERA_X + 1.6 * TARGET_RANGE, CAMERA_Y);
    Spin(100ms);
    contacts_.clear();
    for (int i = 0; i < 2; ++i) {
        TrackFrames({0.0}, 1, {1.6 * TARGET_RANGE});
        TrackFrames({0.0}, 7);
    }
    TrackFrames({0.0}, 10);

    EXPECT_TRUE(released_.empty());
    ASSERT_FALSE(contacts_.empty());
    for (const auto& contact : contacts_) {
        EXPECT_EQ(rclcpp::Time(contact.reports.front().track_start), track);
        EXPECT_NEAR(contact.reports.front().position.x, CAMERA_X + TARGET_RANGE, 1.0);
    }
}

TEST_F(ObjectTrackerToiContactTest, LidarTrackPassingNearAnInvestigatedTargetTwiceIsNotReleased) {
    SetReleaseTime(0.5);
    PublishCameraInfo();
    Spin(100ms);
    TrackFrames({0.0}, 20, {}, {CAMERA_X + TARGET_RANGE, CAMERA_Y});
    ASSERT_FALSE(contacts_.empty());
    const auto track = rclcpp::Time(contacts_.back().reports.front().track_start);

    PublishInvestigated(CAMERA_X + TARGET_RANGE, CAMERA_Y + 6.0);
    Spin(100ms);
    contacts_.clear();
    const auto lidar_frames = [this](double y, int frames) {
        for (int i = 0; i < frames; ++i) {
            PublishObstacle(CAMERA_X + TARGET_RANGE, CAMERA_Y + y);
            Spin(100ms);
        }
    };
    for (int i = 0; i < 2; ++i) {
        lidar_frames(3.0, 2);
        lidar_frames(0.0, 8);
    }

    EXPECT_TRUE(released_.empty());
    ASSERT_FALSE(contacts_.empty());
    EXPECT_EQ(rclcpp::Time(contacts_.back().reports.front().track_start), track);
}

TEST_F(ObjectTrackerToiContactTest, TrackWhoseDetectionsAreAcceptedIsNotReleasedWhileItsEstimateLags) {
    SetReleaseTime(0.2);
    tracking_node_->set_parameter(rclcpp::Parameter("target_selection.investigated_toi_radius", 5.0));
    PublishCameraInfo();
    Spin(100ms);
    TrackFrames({0.0}, 20);
    ASSERT_FALSE(contacts_.empty());
    const auto track = rclcpp::Time(contacts_.back().reports.front().track_start);

    PublishInvestigated(CAMERA_X + TARGET_RANGE + 7.0, CAMERA_Y + 0.5);
    Spin(100ms);
    contacts_.clear();
    for (int i = 0; i < 20; ++i) {
        TrackFrames({0.0}, 1, {TARGET_RANGE + 15.0});
    }

    EXPECT_TRUE(released_.empty());
    ASSERT_FALSE(contacts_.empty());
    EXPECT_EQ(rclcpp::Time(contacts_.back().reports.front().track_start), track);
    EXPECT_NEAR(contacts_.back().reports.front().position.x, CAMERA_X + TARGET_RANGE + 15.0, 1.5);
}

TEST_F(ObjectTrackerToiContactTest, LidarLockOnAnInvestigatedTargetIsReleasedDespiteAnOffRangeDetection) {
    SetReleaseTime(0.5);
    PublishCameraInfo();
    Spin(100ms);
    TrackFrames({0.0}, 20, {}, {CAMERA_X + TARGET_RANGE, CAMERA_Y});
    ASSERT_FALSE(contacts_.empty());

    PublishInvestigated(CAMERA_X + TARGET_RANGE, CAMERA_Y + 2.0);
    Spin(100ms);
    TrackFrames({3.0}, 15, {TARGET_RANGE + 10.0}, {CAMERA_X + TARGET_RANGE, CAMERA_Y});
    EXPECT_EQ(released_.size(), 1u);
}

TEST_F(ObjectTrackerToiContactTest, TrackThatLostItsLidarBoxIsNotReleasedWhileItsDetectionsAreAccepted) {
    SetReleaseTime(0.2);
    tracking_node_->set_parameter(rclcpp::Parameter("target_selection.investigated_toi_radius", 5.0));
    PublishCameraInfo();
    Spin(100ms);
    TrackFrames({0.0}, 20, {}, {CAMERA_X + TARGET_RANGE, CAMERA_Y});
    ASSERT_FALSE(contacts_.empty());
    const auto track = rclcpp::Time(contacts_.back().reports.front().track_start);

    PublishInvestigated(CAMERA_X + TARGET_RANGE + 7.0, CAMERA_Y + 0.5);
    Spin(100ms);
    contacts_.clear();
    for (int i = 0; i < 20; ++i) {
        PublishObstacles({{9, CAMERA_X - 50.0, CAMERA_Y - 50.0}});
        PublishDetections({0.0}, {TARGET_RANGE + 15.0});
        Spin(100ms);
    }

    EXPECT_TRUE(released_.empty());
    ASSERT_FALSE(contacts_.empty());
    EXPECT_EQ(rclcpp::Time(contacts_.back().reports.front().track_start), track);
    EXPECT_NEAR(contacts_.back().reports.front().position.x, CAMERA_X + TARGET_RANGE + 15.0, 1.5);
}

TEST_F(ObjectTrackerToiContactTest, TrackWhoseDetectionLandsOnAnInvestigatedTargetReportsItThere) {
    PublishCameraInfo();
    Spin(100ms);
    TrackFrames({0.0}, 20, {1.6 * TARGET_RANGE});
    ASSERT_FALSE(contacts_.empty());
    ASSERT_GT(contacts_.back().reports.front().position.x, CAMERA_X + 1.4 * TARGET_RANGE);
    const int32_t track_sec = contacts_.back().reports.front().track_start.sec;
    contacts_.clear();

    PublishInvestigated(CAMERA_X + TARGET_RANGE, CAMERA_Y);
    TrackFrames({0.0}, 8, {1.6 * TARGET_RANGE}, {CAMERA_X + TARGET_RANGE, CAMERA_Y});

    ASSERT_FALSE(contacts_.empty());
    const auto& last = contacts_.back().reports.front();
    EXPECT_EQ(last.track_start.sec, track_sec);
    EXPECT_NEAR(last.position.x, CAMERA_X + TARGET_RANGE, 1.0);
    EXPECT_NEAR(last.position.y, CAMERA_Y, 1.0);
}

TEST_F(ObjectTrackerToiContactTest, LidarTrackedTargetHiddenByAnInvestigatedOneInFrontKeepsItsTrack) {
    constexpr double far = 30.0;
    constexpr double near = 15.0;
    PublishCameraInfo();
    PublishInvestigated(CAMERA_X + near, CAMERA_Y + 0.3);
    Spin(100ms);
    TrackFrames({0.0}, 20, {far}, {CAMERA_X + far, CAMERA_Y});
    ASSERT_FALSE(contacts_.empty());
    const int32_t track_sec = contacts_.back().reports.front().track_start.sec;
    contacts_.clear();

    for (int i = 0; i < 20; ++i) {
        PublishObstacles({{9, CAMERA_X + near, CAMERA_Y + 0.3}, {7, CAMERA_X + far, CAMERA_Y}});
        PublishDetections({0.3}, {near});
        Spin(100ms);
    }

    ASSERT_FALSE(contacts_.empty());
    for (const auto& contact : contacts_) {
        const auto& p = contact.reports.front();
        EXPECT_EQ(p.track_start.sec, track_sec);
        EXPECT_NEAR(p.position.x, CAMERA_X + far, 1.0);
        EXPECT_NEAR(p.position.y, CAMERA_Y, 1.0);
    }
}

TEST_F(ObjectTrackerToiContactTest, CameraTrackedTargetHiddenByAMuchNearerInvestigatedOneIsNotReleasedThere) {
    constexpr double far = 40.0;
    constexpr double near = 15.0;
    PublishCameraInfo();
    PublishInvestigated(CAMERA_X + near, CAMERA_Y);
    Spin(100ms);
    TrackFrames({0.0}, 20, {far});
    ASSERT_FALSE(contacts_.empty());
    contacts_.clear();

    for (int i = 0; i < 20; ++i) {
        PublishObstacles({{9, CAMERA_X + near, CAMERA_Y}});
        PublishDetections({0.0}, {near});
        Spin(100ms);
    }

    EXPECT_TRUE(released_.empty());
    for (const auto& contact : contacts_) {
        EXPECT_GT(contact.reports.front().position.x, CAMERA_X + 0.75 * far);
    }
}

TEST_F(ObjectTrackerToiContactTest, InvestigatedMarkerBesideACameraTrackedTargetDoesNotReleaseIt) {
    PublishCameraInfo();
    Spin(100ms);
    TrackFrames({0.0}, 20);
    ASSERT_FALSE(contacts_.empty());
    const auto track = contacts_.back().reports.front().track_start;
    contacts_.clear();

    // An investigated target 8 m nearer projects about three detection
    // half-diagonals to the side of the tracked target's box.
    const Obstacle investigated{9, CAMERA_X + 12.0, CAMERA_Y - 3.0};
    PublishInvestigated(investigated.x, investigated.y);
    Spin(100ms);
    for (int i = 0; i < 30; ++i) {
        PublishObstacles({investigated});
        PublishDetections({0.0});
        Spin(100ms);
    }

    EXPECT_GE(contacts_.size(), 2u);
    for (const auto& contact : contacts_) {
        EXPECT_EQ(rclcpp::Time(contact.reports.front().track_start), rclcpp::Time(track));
        const auto& p = contact.reports.front().position;
        EXPECT_NEAR(p.x, CAMERA_X + TARGET_RANGE, 1.0);
        EXPECT_NEAR(p.y, CAMERA_Y, 1.0);
    }
}

TEST_F(ObjectTrackerToiContactTest, InvestigatedTargetAloneIsNotTracked) {
    PublishCameraInfo();
    PublishInvestigated(CAMERA_X + TARGET_RANGE, CAMERA_Y);
    Spin(100ms);
    TrackFrames({0.0}, 20);
    EXPECT_TRUE(contacts_.empty());
}

TEST_F(ObjectTrackerToiContactTest, ContactsOfOneTrackShareItsStartStamp) {
    PublishCameraInfo();
    Spin(100ms);
    TrackFrames({0.0, OTHER_Y_OFFSET}, 20);
    ASSERT_GE(contacts_.size(), 2u);
    const auto first_track = rclcpp::Time(contacts_.front().reports.front().track_start);
    EXPECT_GT(first_track.nanoseconds(), 0);
    for (const auto& contact : contacts_) {
        EXPECT_EQ(rclcpp::Time(contact.reports.front().track_start), first_track);
    }

    PublishInvestigated(CAMERA_X + TARGET_RANGE, CAMERA_Y);
    Spin(800ms);
    contacts_.clear();
    TrackFrames({0.0, OTHER_Y_OFFSET}, 30);
    ASSERT_GE(contacts_.size(), 2u);
    const auto second_track = rclcpp::Time(contacts_.front().reports.front().track_start);
    EXPECT_NE(second_track, first_track);
    for (const auto& contact : contacts_) {
        EXPECT_EQ(rclcpp::Time(contact.reports.front().track_start), second_track);
    }
}

TEST_F(ObjectTrackerToiContactTest, InvestigatedTargetWithAnOffRangeEstimateIsStillSkipped) {
    PublishCameraInfo();
    PublishInvestigated(CAMERA_X + TARGET_RANGE, CAMERA_Y);
    Spin(100ms);
    TrackFrames({0.0, OTHER_Y_OFFSET}, 30, {1.5 * TARGET_RANGE, TARGET_RANGE});

    ASSERT_FALSE(contacts_.empty());
    for (const auto& contact : contacts_) {
        const auto& p = contact.reports.front().position;
        EXPECT_NEAR(p.x, CAMERA_X + TARGET_RANGE, 0.5);
        EXPECT_NEAR(p.y, CAMERA_Y + OTHER_Y_OFFSET, 0.5);
    }
}

TEST_F(ObjectTrackerToiContactTest, NewTargetInLineBeyondAnInvestigatedOneIsPickedUp) {
    PublishCameraInfo();
    PublishInvestigated(CAMERA_X + TARGET_RANGE, CAMERA_Y);
    Spin(100ms);
    TrackFrames({0.0}, 20, {TARGET_RANGE + 25.0});

    ASSERT_FALSE(contacts_.empty());
    for (const auto& contact : contacts_) {
        const auto& p = contact.reports.front().position;
        EXPECT_NEAR(p.x, CAMERA_X + TARGET_RANGE + 25.0, 0.5);
        EXPECT_NEAR(p.y, CAMERA_Y, 0.5);
    }
}

TEST_F(ObjectTrackerToiContactTest, TrackedTargetPassingInFrontOfAnInvestigatedOneKeepsItsDetections) {
    constexpr double range = 12.0;
    PublishCameraInfo();
    PublishInvestigated(CAMERA_X + TARGET_RANGE, CAMERA_Y);
    Spin(100ms);
    TrackFrames({-4.0}, 20, {range});
    ASSERT_FALSE(contacts_.empty());

    for (int i = 1; i <= 20; ++i) {
        TrackFrames({-4.0 + 0.2 * i}, 1, {range});
    }
    contacts_.clear();
    TrackFrames({0.0}, 20, {range});

    ASSERT_FALSE(contacts_.empty());
    const auto& p = contacts_.back().reports.front().position;
    EXPECT_NEAR(p.x, CAMERA_X + range, 0.5);
    EXPECT_NEAR(p.y, CAMERA_Y, 0.5);
}

// The tracked target A, held by its LiDAR box, leaves the camera's view when
// the camera turns 50 deg left; another target B of the same class 6.9 m from
// A is then seen 30 deg to the right, 20 m away.
// Ranging of the reports (avt_341_msgs/TargetReport)
// -----------------------------------------------------------------------------

constexpr uint8_t LIDAR_RANGED = avt_341_msgs::msg::TargetReport::RANGING_LIDAR;
constexpr uint8_t CAMERA_RANGED = avt_341_msgs::msg::TargetReport::RANGING_CAMERA;

TEST_F(ObjectTrackerToiContactTest, LockedTrackReportsItsLidarBoxFromTheCameraPosition) {
    PublishCameraInfo();
    Spin(100ms);
    TrackFrames({0.0}, 20, {}, {CAMERA_X + TARGET_RANGE, CAMERA_Y});

    ASSERT_FALSE(contacts_.empty());
    const auto& report = contacts_.back().reports.front();
    EXPECT_EQ(report.ranging, LIDAR_RANGED);
    EXPECT_NEAR(report.observer.x, CAMERA_X, 1e-6);
    EXPECT_NEAR(report.observer.y, CAMERA_Y, 1e-6);
}

TEST_F(ObjectTrackerToiContactTest, BoxOfImplausibleSizeDoesNotRangeTheTarget) {
    PublishCameraInfo();
    Spin(100ms);
    for (int i = 0; i < 20; ++i) {
        PublishObstacles({{7, CAMERA_X + TARGET_RANGE, CAMERA_Y, 12.0, 4.0}});
        PublishDetections({0.0});
        Spin(100ms);
    }

    ASSERT_FALSE(contacts_.empty());
    EXPECT_EQ(contacts_.back().reports.front().ranging, CAMERA_RANGED);
}

TEST_F(ObjectTrackerToiContactTest, BoxFarBeyondTheDetectionsCameraRangeDoesNotRangeTheTarget) {
    PublishCameraInfo();
    Spin(100ms);
    TrackFrames({0.0}, 20, {}, {CAMERA_X + TARGET_RANGE + 15.0, CAMERA_Y});

    ASSERT_FALSE(contacts_.empty());
    EXPECT_NEAR(contacts_.back().reports.front().position.x, CAMERA_X + TARGET_RANGE + 15.0, 1.0);
    EXPECT_EQ(contacts_.back().reports.front().ranging, CAMERA_RANGED);
}

TEST_F(ObjectTrackerToiContactTest, LockThatCarriesOnPastADetectionElsewhereIsCameraRanged) {
    PublishCameraInfo();
    Spin(100ms);
    TrackFrames({0.0}, 20, {}, {CAMERA_X + TARGET_RANGE, CAMERA_Y});
    ASSERT_EQ(contacts_.back().reports.front().ranging, LIDAR_RANGED);

    contacts_.clear();
    TrackFrames({3.0}, 15, {}, {CAMERA_X + TARGET_RANGE, CAMERA_Y});
    ASSERT_FALSE(contacts_.empty());
    EXPECT_EQ(contacts_.back().reports.front().ranging, CAMERA_RANGED);
}

TEST_F(ObjectTrackerToiContactTest, FilteredPositionAwayFromTheLidarBoxIsCameraRanged) {
    PublishCameraInfo();
    Spin(100ms);
    TrackFrames({0.0}, 20, {}, {CAMERA_X + TARGET_RANGE, CAMERA_Y});
    ASSERT_EQ(contacts_.back().reports.front().ranging, LIDAR_RANGED);

    contacts_.clear();
    TrackFrames({0.0}, 15, {TARGET_RANGE + 15.0}, {CAMERA_X + TARGET_RANGE + 15.0, CAMERA_Y});
    ASSERT_FALSE(contacts_.empty());
    EXPECT_NEAR(contacts_.back().reports.front().position.x, CAMERA_X + TARGET_RANGE, 2.0);
    EXPECT_EQ(contacts_.back().reports.front().ranging, CAMERA_RANGED);
}

TEST_F(ObjectTrackerToiContactTest, LockedBoxThatWalksAwayFromItsLastMatchIsCameraRanged) {
    tracking_node_->set_parameter(rclcpp::Parameter("target_selection.investigated_toi_radius", 5.0));
    PublishCameraInfo();
    Spin(100ms);
    TrackFrames({0.0}, 20, {}, {CAMERA_X + TARGET_RANGE, CAMERA_Y});
    ASSERT_EQ(contacts_.back().reports.front().ranging, LIDAR_RANGED);

    contacts_.clear();
    for (int i = 1; i <= 60; ++i) {
        PublishObstacle(CAMERA_X + TARGET_RANGE + 0.2 * std::min(i, 35), CAMERA_Y);
        Spin(100ms);
    }
    ASSERT_FALSE(contacts_.empty());
    EXPECT_NEAR(contacts_.back().reports.front().position.x, CAMERA_X + TARGET_RANGE + 7.0, 0.5);
    EXPECT_EQ(contacts_.back().reports.front().ranging, CAMERA_RANGED);
}

TEST_F(ObjectTrackerToiContactTest, SightingOnAMatchingBoxIsLidarRanged) {
    PublishCameraInfo();
    Spin(100ms);
    for (int i = 0; i < 30; ++i) {
        PublishObstacles({{7, CAMERA_X + TARGET_RANGE, CAMERA_Y}, {8, CAMERA_X + TARGET_RANGE, CAMERA_Y + OTHER_Y_OFFSET}});
        PublishDetections({0.0, OTHER_Y_OFFSET});
        Spin(100ms);
    }

    ASSERT_FALSE(sightings_.empty());
    const auto& sighting = sightings_.back().reports.front();
    EXPECT_EQ(sighting.ranging, LIDAR_RANGED);
    EXPECT_NEAR(sighting.position.y, CAMERA_Y + OTHER_Y_OFFSET, 0.5);
}

TEST_F(ObjectTrackerToiContactTest, SightingOnABoxThreeHalfDiagonalsAwayIsCameraRanged) {
    tracking_node_->set_parameter(rclcpp::Parameter("target_selection.investigated_toi_radius", 5.0));
    PublishCameraInfo();
    Spin(100ms);
    for (int i = 0; i < 30; ++i) {
        PublishObstacles({{7, CAMERA_X + TARGET_RANGE, CAMERA_Y}, {8, CAMERA_X + TARGET_RANGE, CAMERA_Y - 5.0}});
        PublishDetections({0.0, OTHER_Y_OFFSET});
        Spin(100ms);
    }

    ASSERT_FALSE(sightings_.empty());
    const auto& sighting = sightings_.back().reports.front();
    EXPECT_EQ(sighting.ranging, CAMERA_RANGED);
    EXPECT_NEAR(sighting.position.y, CAMERA_Y + OTHER_Y_OFFSET, 0.5);
}

TEST_F(ObjectTrackerToiContactTest, DetectionWithinTheBorderMarginIsIgnored) {
    PublishCameraInfo();
    Spin(100ms);
    // Box left edge 2 px from the image border (camera.border_margin defaults to 4).
    TrackFrames({13.775}, 20);
    EXPECT_TRUE(contacts_.empty());
}

TEST_F(ObjectTrackerToiContactTest, LidarLockedTrackReleasedAtAnInvestigatedTargetIsReleasedLidarRanged) {
    SetReleaseTime(0.2);
    PublishCameraInfo();
    Spin(100ms);
    TrackFrames({0.0}, 20, {}, {CAMERA_X + TARGET_RANGE, CAMERA_Y});

    PublishInvestigated(CAMERA_X + TARGET_RANGE, CAMERA_Y + 1.0);
    Spin(100ms);
    TrackFrames({0.0}, 10, {}, {CAMERA_X + TARGET_RANGE, CAMERA_Y});
    ASSERT_EQ(released_.size(), 1u);
    EXPECT_EQ(released_.front().reports.front().ranging, LIDAR_RANGED);
}

class ObjectTrackerToiOutOfViewTest : public ObjectTrackerToiContactTest {
protected:
    void TrackThenTurn() {
        PublishCameraInfo();
        Spin(100ms);
        TrackFrames({0.0}, 20, {}, A);
        ASSERT_FALSE(contacts_.empty());
        PublishCameraTf(-0.664463, 0.241845, -0.241845, 0.664463);
        Spin(200ms);
        contacts_.clear();
    }

    void SeeB(int frames) {
        const double depth = TARGET_RANGE * std::cos(30.0 * M_PI / 180.0);
        TrackFrames({-depth * std::tan(30.0 * M_PI / 180.0)}, frames, {depth}, A);
    }

    static bool AtA(const avt_341_msgs::msg::TargetReports& contact) {
        const auto& p = contact.reports.front().position;
        return std::hypot(p.x - A[0], p.y - A[1]) < 1.0;
    }

    static inline const std::vector<double> A = {CAMERA_X + TARGET_RANGE, CAMERA_Y};
};

TEST_F(ObjectTrackerToiOutOfViewTest, TargetThatLeavesTheImageKeepsItsTrackWhileLidarSeesIt) {
    TrackThenTurn();
    SeeB(90);  // longer than target_timeout

    ASSERT_FALSE(contacts_.empty());
    for (const auto& contact : contacts_) {
        EXPECT_TRUE(AtA(contact));
    }
}

TEST_F(ObjectTrackerToiOutOfViewTest, OtherTargetSeenWhileTrackingIsReportedAsASighting) {
    TrackThenTurn();
    SeeB(20);

    ASSERT_FALSE(contacts_.empty());
    for (const auto& contact : contacts_) {
        EXPECT_TRUE(AtA(contact));
    }
    ASSERT_GE(sightings_.size(), 2u);
    for (const auto& sighting : sightings_) {
        ASSERT_EQ(sighting.reports.size(), 1u);
        EXPECT_NEAR(sighting.reports[0].position.x, CAMERA_X + 18.793852, 1.0);
        EXPECT_NEAR(sighting.reports[0].position.y, CAMERA_Y + 6.840403, 1.0);
    }
}

TEST_F(ObjectTrackerToiContactTest, InvestigatedTargetIsNotReportedAsASighting) {
    PublishCameraInfo();
    PublishInvestigated(CAMERA_X + TARGET_RANGE, CAMERA_Y);
    Spin(100ms);
    TrackFrames({0.0, OTHER_Y_OFFSET}, 30, {1.5 * TARGET_RANGE, TARGET_RANGE});
    ASSERT_FALSE(contacts_.empty());
    EXPECT_TRUE(sightings_.empty());
}

TEST_F(ObjectTrackerToiOutOfViewTest, CameraCorrectsLidarDriftWhenTheTargetIsBackInView) {
    const Obstacle car{7, A[0], A[1]};
    const Obstacle neighbor{9, A[0], A[1] - 2.5};
    const Obstacle car_relabeled{10, A[0], A[1]};
    TrackThenTurn();
    for (int i = 0; i < 10; ++i) {
        PublishObstacles({car});
        Spin(100ms);
    }
    // The car's box is missing for one scan and the neighbor's is re-acquired.
    PublishObstacles({neighbor});
    Spin(100ms);
    for (int i = 0; i < 20; ++i) {
        PublishObstacles({neighbor, car_relabeled});
        Spin(100ms);
    }
    ASSERT_FALSE(contacts_.empty());
    EXPECT_NEAR(contacts_.back().reports.front().position.y, neighbor.y, 0.5);

    // The car is back in view, its relabeled box next to the neighbor's.
    PublishCameraTf(-0.5, 0.5, -0.5, 0.5);
    Spin(200ms);
    contacts_.clear();
    for (int i = 0; i < 15; ++i) {
        PublishObstacles({neighbor, car_relabeled});
        PublishDetections({0.0});
        Spin(100ms);
    }
    ASSERT_FALSE(contacts_.empty());
    for (const auto& contact : contacts_) {
        EXPECT_TRUE(AtA(contact));
    }
}

TEST_F(ObjectTrackerToiOutOfViewTest, RelabeledBoxOfAMovingTargetIsReacquiredWhereTheTargetIs) {
    const Obstacle decoy{11, 0.5, 0.0};
    TrackThenTurn();
    double y = A[1];
    for (int i = 0; i < 40; ++i) {
        y -= 0.2;
        PublishObstacles({{7, A[0], y}});
        Spin(100ms);
    }
    contacts_.clear();
    for (int i = 0; i < 40; ++i) {
        y -= 0.2;
        PublishObstacles({decoy, {10, A[0], y}});
        Spin(100ms);
    }
    ASSERT_GE(contacts_.size(), 3u);
    for (const auto& contact : contacts_) {
        EXPECT_NEAR(contact.reports.front().position.x, A[0], 1.5);
    }
}

TEST_F(ObjectTrackerToiOutOfViewTest, LidarLockHoldsWhileOnlyAPartOfTheTargetIsInView) {
    PublishCameraInfo();
    Spin(100ms);
    TrackFrames({0.0}, 15);
    TrackFrames({0.0}, 5, {}, A);  // first LiDAR lock re-initializes the filter
    // Turn the camera 40 deg right: the target's box now projects just left of
    // the image while a partial detection of it, with a range estimate 50 %
    // long, stays at its left edge.
    PublishCameraTf(-0.298836, 0.640856, -0.640856, 0.298836);
    contacts_.clear();
    TrackFrames({21.1}, 30, {1.5 * TARGET_RANGE}, A);

    ASSERT_FALSE(contacts_.empty());
    for (const auto& contact : contacts_) {
        EXPECT_TRUE(AtA(contact));
    }
}

TEST_F(ObjectTrackerToiOutOfViewTest, TrackedTargetDoesNotLatchOntoANearerObstacleWhileItsBoxIsMissing) {
    PublishCameraInfo();
    Spin(100ms);
    TrackFrames({0.0}, 15);
    TrackFrames({0.0}, 10, {}, A);
    // The target's box drops out while the camera still sees it; an obstacle
    // 8 m nearer projects about three detection half-diagonals to the side.
    const Obstacle nearer{9, CAMERA_X + 12.0, CAMERA_Y - 3.0};
    contacts_.clear();
    for (int i = 0; i < 80; ++i) {
        PublishObstacles({nearer});
        PublishDetections({0.0});
        Spin(100ms);
    }

    ASSERT_FALSE(contacts_.empty());
    for (const auto& contact : contacts_) {
        EXPECT_TRUE(AtA(contact));
    }
}

TEST_F(ObjectTrackerToiContactTest, NewTargetInLineAtASimilarRangeIsSkippedWithoutLidar) {
    PublishCameraInfo();
    PublishInvestigated(CAMERA_X + TARGET_RANGE, CAMERA_Y);
    Spin(100ms);
    TrackFrames({0.0}, 20, {1.5 * TARGET_RANGE});
    EXPECT_TRUE(contacts_.empty());
}

TEST_F(ObjectTrackerToiContactTest, FirstContactWaitsForSeveralCameraDetections) {
    PublishCameraInfo();
    Spin(100ms);
    TrackFrames({0.0}, 2);
    Spin(500ms);
    EXPECT_TRUE(contacts_.empty());

    TrackFrames({0.0}, 1);
    Spin(200ms);
    EXPECT_FALSE(contacts_.empty());
}

TEST_F(ObjectTrackerToiContactTest, DetectionOfTheEgoHoodStartsNoTrack) {
    PublishCameraInfo();
    Spin(100ms);
    for (int i = 0; i < 20; ++i) {
        vision_msgs::msg::Detection2DArray detections;
        detections.header.stamp = tracking_node_->get_clock()->now();
        detections.header.frame_id = CAMERA_FRAME;
        vision_msgs::msg::Detection2D detection;
        detection.bbox.center.position.x = IMAGE_CX;
        detection.bbox.center.position.y = 449.0;
        detection.bbox.size_x = 500.0;
        detection.bbox.size_y = 115.0;
        vision_msgs::msg::ObjectHypothesisWithPose hypothesis;
        hypothesis.hypothesis.class_id = TARGET;
        hypothesis.hypothesis.score = 0.7;
        detection.results.push_back(hypothesis);
        detections.detections.push_back(detection);
        detections_pub_->publish(detections);
        Spin(100ms);
    }
    EXPECT_TRUE(contacts_.empty());
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    std::vector<const char*> args(argv, argv + argc);
    args.insert(args.end(), {"--ros-args", "-p", "frames.use_vehicle_prefix:=false",
                             "-p", "frames.camera:=test_camera",
                             "-p", "obstacle_detector.use_external_detector:=true",
                             "-p", "camera.ego_mask_top:=0.7"});
    rclcpp::init(static_cast<int>(args.size()), args.data());
    int result = RUN_ALL_TESTS();
    rclcpp::shutdown();
    return result;
}
