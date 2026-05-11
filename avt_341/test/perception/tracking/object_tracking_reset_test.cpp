// Unit tests for the object tracking node's reset feature.
//
// The reset protocol: a publisher sends "perception" on avt_341/reset;
// the node resets its filter state and acknowledges on avt_341/reset_ack.
// Messages addressed to other node types must be silently ignored.

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <avt_341/node/node_proxy.h>
#include <avt_341/perception/tracking/object_tracking_node.hpp>

#include <chrono>
#include <functional>

using namespace std::chrono_literals;

class ObjectTrackingResetTest : public ::testing::Test {
protected:
    void SetUp() override {
        tracking_node_ = std::make_shared<avt_341::perception::ObjectTrackingNode>();
        helper_node_ = rclcpp::Node::make_shared("reset_test_helper");

        reset_pub_ = helper_node_->create_publisher<std_msgs::msg::String>(
            "avt_341/reset", 10);

        ack_received_ = false;
        ack_sub_ = helper_node_->create_subscription<std_msgs::msg::String>(
            "avt_341/reset_ack", 10,
            [this](std_msgs::msg::String::SharedPtr msg) {
                if (msg->data == avt_341::node::NodeType::Perception) {
                    ack_received_ = true;
                }
            });

        exec_.add_node(tracking_node_);
        exec_.add_node(helper_node_);
    }

    void TearDown() override {
        exec_.remove_node(tracking_node_);
        exec_.remove_node(helper_node_);
    }

    // Spin until condition() returns true or timeout elapses.
    void SpinUntil(std::function<bool()> condition,
                   std::chrono::milliseconds timeout) {
        auto deadline = std::chrono::steady_clock::now() + timeout;
        while (!condition() && std::chrono::steady_clock::now() < deadline) {
            exec_.spin_some(10ms);
        }
    }

    std::shared_ptr<avt_341::perception::ObjectTrackingNode> tracking_node_;
    std::shared_ptr<rclcpp::Node> helper_node_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr reset_pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr ack_sub_;
    rclcpp::executors::SingleThreadedExecutor exec_;
    bool ack_received_ = false;
};

// A "perception" reset must be acknowledged on avt_341/reset_ack.
TEST_F(ObjectTrackingResetTest, PerceptionResetTriggersAck) {
    // Let subscriptions settle before publishing.
    SpinUntil([] { return false; }, 100ms);

    std_msgs::msg::String msg;
    msg.data = avt_341::node::NodeType::Perception;
    reset_pub_->publish(msg);

    SpinUntil([this] { return ack_received_; }, 2000ms);

    EXPECT_TRUE(ack_received_);
}

// A reset addressed to a different node type must not trigger the perception ack.
TEST_F(ObjectTrackingResetTest, NonPerceptionResetIsIgnored) {
    SpinUntil([] { return false; }, 100ms);

    std_msgs::msg::String msg;
    msg.data = avt_341::node::NodeType::GlobalPlanner;
    reset_pub_->publish(msg);

    SpinUntil([] { return false; }, 500ms);

    EXPECT_FALSE(ack_received_);
}

// A second reset after the first must also be acknowledged (flag is cleared correctly).
TEST_F(ObjectTrackingResetTest, RepeatedResetsAreAcknowledged) {
    SpinUntil([] { return false; }, 100ms);

    std_msgs::msg::String msg;
    msg.data = avt_341::node::NodeType::Perception;

    for (int i = 0; i < 3; ++i) {
        ack_received_ = false;
        reset_pub_->publish(msg);
        SpinUntil([this] { return ack_received_; }, 2000ms);
        EXPECT_TRUE(ack_received_) << "Reset " << (i + 1) << " not acknowledged";
    }
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    rclcpp::init(argc, argv);
    int result = RUN_ALL_TESTS();
    rclcpp::shutdown();
    return result;
}
