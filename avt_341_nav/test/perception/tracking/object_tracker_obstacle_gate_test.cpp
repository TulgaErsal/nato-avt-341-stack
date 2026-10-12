// Unit tests for when the object tracking node feeds obstacle markers to its
// trackers.
//
// The markers are only produced while a tracker can use them: once a tracker
// has had its first camera detection (or with run_without_trackers). A tracker
// that exists but was never detected must not cost an obstacle detection.
// Built twice: with the integrated obstacle detector, and with
// OBJECT_TRACKER_TEST_EXTERNAL_DETECTOR, where the markers come from
// lidar_obstacle_detector_node and are used only once the obstacle cloud of
// the same scan has arrived.

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <avt_341_msgs/msg/mission_module_status.hpp>
#include <avt_341_nav/perception/tracking/object_tracker_node.hpp>

#include <chrono>
#include <functional>
#include <string>
#include <vector>

using namespace std::chrono_literals;

namespace {

const std::string TARGET = "TI_box";

sensor_msgs::msg::PointCloud2 MakeCloud(const builtin_interfaces::msg::Time& stamp) {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    for (int i = 0; i < 10; ++i) {
        for (int j = 0; j < 10; ++j) {
            cloud.push_back(pcl::PointXYZ(5.0f, 0.1f * i, 0.1f * j));
        }
    }
    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(cloud, msg);
    msg.header.stamp = stamp;
    msg.header.frame_id = "base_link";
    return msg;
}

visualization_msgs::msg::MarkerArray MakeMarkers(
    const builtin_interfaces::msg::Time& stamp, bool with_box) {
    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker delete_all;
    delete_all.action = visualization_msgs::msg::Marker::DELETEALL;
    delete_all.header.stamp = stamp;
    delete_all.header.frame_id = "base_link";
    markers.markers.push_back(delete_all);
    if (with_box) {
        visualization_msgs::msg::Marker box;
        box.header = delete_all.header;
        box.ns = "lidar_bboxes";
        box.id = 1;
        box.type = visualization_msgs::msg::Marker::CUBE;
        box.action = visualization_msgs::msg::Marker::ADD;
        box.pose.position.x = 5.0;
        box.pose.position.y = 0.5;
        box.pose.position.z = 0.5;
        box.pose.orientation.w = 1.0;
        box.scale.x = 0.2;
        box.scale.y = 1.0;
        box.scale.z = 1.0;
        markers.markers.push_back(box);
    }
    return markers;
}

builtin_interfaces::msg::Time Stamp(int32_t sec) {
    builtin_interfaces::msg::Time stamp;
    stamp.sec = sec;
    return stamp;
}

}  // namespace

class ObjectTrackerObstacleGateTest : public ::testing::Test {
protected:
    void SetUp() override {
        tracking_node_ = std::make_shared<avt_341_nav::perception::ObjectTrackerNode>();
        helper_node_ = rclcpp::Node::make_shared("obstacle_gate_test_helper");

        task_pub_ = helper_node_->create_publisher<avt_341_msgs::msg::MissionModuleStatus>("task", 10);
        camera_info_pub_ = helper_node_->create_publisher<sensor_msgs::msg::CameraInfo>("camera_info", 10);
        detections_pub_ = helper_node_->create_publisher<vision_msgs::msg::Detection2DArray>("detection_2d", 10);
        points_pub_ = helper_node_->create_publisher<sensor_msgs::msg::PointCloud2>("points/input", 10);
        external_bboxes_pub_ = helper_node_->create_publisher<visualization_msgs::msg::MarkerArray>("obstacles/bboxes", 10);
        external_clusters_pub_ = helper_node_->create_publisher<sensor_msgs::msg::PointCloud2>("obstacles/cloud_clusters", 10);

        bboxes_sub_ = helper_node_->create_subscription<visualization_msgs::msg::MarkerArray>(
            "lidar_detector/bboxes", 10,
            [this](visualization_msgs::msg::MarkerArray::SharedPtr msg) {
                received_.push_back(*msg);
            });

        exec_.add_node(tracking_node_);
        exec_.add_node(helper_node_);
        Spin(200ms);
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

    void SelectTarget() {
        avt_341_msgs::msg::MissionModuleStatus status;
        status.active_task.tracked_vehicle = TARGET;
        task_pub_->publish(status);
        Spin(200ms);
    }

    void DetectTarget() {
        sensor_msgs::msg::CameraInfo camera_info;
        camera_info.width = 640;
        camera_info.height = 480;
        camera_info_pub_->publish(camera_info);
        Spin(100ms);

        vision_msgs::msg::Detection2DArray detections;
        vision_msgs::msg::Detection2D detection;
        detection.bbox.center.position.x = 320.0;
        detection.bbox.center.position.y = 240.0;
        detection.bbox.size_x = 100.0;
        detection.bbox.size_y = 80.0;
        vision_msgs::msg::ObjectHypothesisWithPose hypothesis;
        hypothesis.hypothesis.class_id = TARGET;
        hypothesis.hypothesis.score = 0.9;
        detection.results.push_back(hypothesis);
        detections.detections.push_back(detection);
        detections_pub_->publish(detections);
        Spin(200ms);
    }

    // Feeds a few scans to whichever obstacle source the node is built for.
    void FeedScans() {
        for (int32_t sec = 1; sec <= 5; ++sec) {
#ifdef OBJECT_TRACKER_TEST_EXTERNAL_DETECTOR
            external_clusters_pub_->publish(MakeCloud(Stamp(sec)));
            external_bboxes_pub_->publish(MakeMarkers(Stamp(sec), true));
#else
            points_pub_->publish(MakeCloud(Stamp(sec)));
#endif
            Spin(100ms);
        }
        Spin(500ms);
    }

    std::shared_ptr<avt_341_nav::perception::ObjectTrackerNode> tracking_node_;
    std::shared_ptr<rclcpp::Node> helper_node_;
    rclcpp::Publisher<avt_341_msgs::msg::MissionModuleStatus>::SharedPtr task_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
    rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr detections_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr points_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr external_bboxes_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr external_clusters_pub_;
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr bboxes_sub_;
    rclcpp::executors::SingleThreadedExecutor exec_;
    std::vector<visualization_msgs::msg::MarkerArray> received_;
};

TEST_F(ObjectTrackerObstacleGateTest, NoTrackerProducesNoMarkers) {
    FeedScans();
    EXPECT_TRUE(received_.empty());
}

TEST_F(ObjectTrackerObstacleGateTest, UndetectedTrackerProducesNoMarkers) {
    SelectTarget();
    FeedScans();
    EXPECT_TRUE(received_.empty());
}

TEST_F(ObjectTrackerObstacleGateTest, DetectedTrackerProducesMarkers) {
    SelectTarget();
    DetectTarget();
    FeedScans();
    EXPECT_FALSE(received_.empty());
}

#ifdef OBJECT_TRACKER_TEST_EXTERNAL_DETECTOR
TEST_F(ObjectTrackerObstacleGateTest, ExternalMarkersWaitForTheirCloud) {
    SelectTarget();
    DetectTarget();

    external_clusters_pub_->publish(MakeCloud(Stamp(1)));
    Spin(100ms);
    external_bboxes_pub_->publish(MakeMarkers(Stamp(2), true));
    Spin(300ms);
    EXPECT_TRUE(received_.empty());

    external_clusters_pub_->publish(MakeCloud(Stamp(2)));
    SpinUntil([this] { return !received_.empty(); }, 1000ms);
    ASSERT_EQ(received_.size(), 1u);
    EXPECT_EQ(received_.front().markers.size(), 2u);
    EXPECT_EQ(received_.front().markers.front().header.stamp.sec, 2);
}

TEST_F(ObjectTrackerObstacleGateTest, ExternalMarkersWithoutBoxesNeedNoCloud) {
    SelectTarget();
    DetectTarget();

    external_bboxes_pub_->publish(MakeMarkers(Stamp(3), false));
    SpinUntil([this] { return !received_.empty(); }, 1000ms);
    ASSERT_EQ(received_.size(), 1u);
    EXPECT_EQ(received_.front().markers.size(), 1u);
}
#endif

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
#ifdef OBJECT_TRACKER_TEST_EXTERNAL_DETECTOR
    std::vector<const char*> args(argv, argv + argc);
    args.insert(args.end(), {"--ros-args", "-p", "obstacle_detector.use_external_detector:=true"});
    rclcpp::init(static_cast<int>(args.size()), args.data());
#else
    rclcpp::init(argc, argv);
#endif
    int result = RUN_ALL_TESTS();
    rclcpp::shutdown();
    return result;
}
