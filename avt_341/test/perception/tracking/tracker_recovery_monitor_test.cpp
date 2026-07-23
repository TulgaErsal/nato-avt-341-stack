// Unit tests for the TrackerRecoveryMonitor lost-detection/recovery advisor.
//
// A helper node hosts mock mission manager services on the absolute names
// the monitor addresses ("/agv1/avt_341/check_speed" and
// "/agv1/avt_341/get_odometry") with settable canned responses and request
// counters. The monitor is driven manually with synthetic TickInputs while a
// single-threaded executor delivers the service traffic, mirroring the
// production setup (both run on the tracking node's executor).

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include <avt_341/perception/tracking/tracker_recovery_monitor.hpp>

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>

using namespace std::chrono_literals;

using avt_341::perception::RecoverySettings;
using avt_341::perception::TrackerRecoveryMonitor;
using avt_341::perception::TrackerState;

class TrackerRecoveryMonitorTest : public ::testing::Test {
protected:
    void SetUp() override {
        host_node_ = rclcpp::Node::make_shared("recovery_monitor_test_host");
        server_node_ = rclcpp::Node::make_shared("mock_mission_manager");

        check_speed_srv_ =
            server_node_->create_service<avt_341_msgs::srv::CheckSpeed>(
                "/agv1/avt_341/check_speed",
                [this](const std::shared_ptr<
                           avt_341_msgs::srv::CheckSpeed::Request>
                           request,
                       std::shared_ptr<avt_341_msgs::srv::CheckSpeed::Response>
                           response) {
                    check_speed_count_++;
                    last_check_speed_request_ = *request;
                    response->is_true = mock_is_moving_;
                });

        get_odometry_srv_ =
            server_node_->create_service<avt_341_msgs::srv::GetOdometry>(
                "/agv1/avt_341/get_odometry",
                [this](const std::shared_ptr<
                           avt_341_msgs::srv::GetOdometry::Request>,
                       std::shared_ptr<avt_341_msgs::srv::GetOdometry::Response>
                           response) {
                    get_odometry_count_++;
                    response->odom = mock_odom_;
                });

        exec_.add_node(host_node_);
        exec_.add_node(server_node_);
    }

    void TearDown() override {
        exec_.remove_node(host_node_);
        exec_.remove_node(server_node_);
    }

    RecoverySettings MakeSettings() const {
        RecoverySettings settings;
        settings.no_movement_threshold = 0.2;
        settings.no_movement_window_time = 0.1;
        settings.no_movement_check_in_states = {
            TrackerState::LIDAR_ONLY_TRACKING,
            TrackerState::CAMERA_ONLY_TRACKING, TrackerState::FULL_TRACKING};
        settings.no_movement_backoff_time = 0.5;
        settings.uncertainty_threshold = 10.0;
        settings.uncertainty_window_time = 0.1;
        return settings;
    }

    std::unique_ptr<TrackerRecoveryMonitor> MakeMonitor(
        const std::string& target_class = "agv1") {
        return std::make_unique<TrackerRecoveryMonitor>(
            host_node_.get(), target_class, MakeSettings(),
            host_node_->get_logger());
    }

    static TrackerRecoveryMonitor::TickInput TrackingInput(
        const double speed, const double variance) {
        TrackerRecoveryMonitor::TickInput input;
        input.state = TrackerState::FULL_TRACKING;
        input.filter_initialized = true;
        input.has_tracked_target = true;
        input.ctr_speed = speed;
        input.xy_covariance = Eigen::Matrix2d::Identity() * variance;
        input.time_since_valid_target = 0.0;
        input.target_timeout = 5.0;
        return input;
    }

    // Measurements starved: no valid measurement for longer than the
    // tracker timeout.
    static TrackerRecoveryMonitor::TickInput StarvedInput() {
        TrackerRecoveryMonitor::TickInput input;
        input.state = TrackerState::NO_DETECTION;
        input.filter_initialized = true;
        input.has_tracked_target = false;
        input.time_since_valid_target = 1.0;
        input.target_timeout = 0.5;
        return input;
    }

    static TrackerRecoveryMonitor::TickInput LostInput() {
        TrackerRecoveryMonitor::TickInput input;
        input.state = TrackerState::LOST;
        return input;
    }

    nav_msgs::msg::Odometry ValidOdom(const double x, const double y) {
        nav_msgs::msg::Odometry odom;
        odom.header.frame_id = "map";
        odom.header.stamp = host_node_->get_clock()->now();
        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.orientation.w = 1.0;
        return odom;
    }

    // Tick the monitor (~every 10 ms, spinning service traffic in between)
    // until condition(result) is true or the timeout elapses. Returns the
    // last UpdateResult.
    TrackerRecoveryMonitor::UpdateResult TickUntil(
        TrackerRecoveryMonitor& monitor,
        const TrackerRecoveryMonitor::TickInput& input,
        std::function<bool(const TrackerRecoveryMonitor::UpdateResult&)>
            condition,
        std::chrono::milliseconds timeout) {
        TrackerRecoveryMonitor::UpdateResult result;
        const auto deadline = std::chrono::steady_clock::now() + timeout;
        while (std::chrono::steady_clock::now() < deadline) {
            result = monitor.Update(input);
            if (condition(result)) {
                break;
            }
            exec_.spin_some(5ms);
            std::this_thread::sleep_for(5ms);
        }
        return result;
    }

    // Tick the monitor for a fixed duration regardless of results.
    void TickFor(TrackerRecoveryMonitor& monitor,
                 const TrackerRecoveryMonitor::TickInput& input,
                 std::chrono::milliseconds duration) {
        TickUntil(
            monitor, input,
            [](const TrackerRecoveryMonitor::UpdateResult&) { return false; },
            duration);
    }

    std::shared_ptr<rclcpp::Node> host_node_;
    std::shared_ptr<rclcpp::Node> server_node_;
    rclcpp::Service<avt_341_msgs::srv::CheckSpeed>::SharedPtr check_speed_srv_;
    rclcpp::Service<avt_341_msgs::srv::GetOdometry>::SharedPtr
        get_odometry_srv_;
    rclcpp::executors::SingleThreadedExecutor exec_;

    int check_speed_count_ = 0;
    int get_odometry_count_ = 0;
    bool mock_is_moving_ = false;
    nav_msgs::msg::Odometry mock_odom_;
    avt_341_msgs::srv::CheckSpeed::Request last_check_speed_request_;
};

// High windowed-mean uncertainty marks the tracker lost directly, without a
// vehicle-to-vehicle confirmation call. The sampled measure is the standard
// deviation along the axis of largest variance: variance 400 -> 20 m, above
// the 10 m threshold.
TEST_F(TrackerRecoveryMonitorTest, UncertaintyAboveThresholdMarksLost) {
    auto monitor = MakeMonitor();

    const auto result = TickUntil(
        *monitor, TrackingInput(5.0, 400.0),
        [](const TrackerRecoveryMonitor::UpdateResult& r) { return r.mark_lost; },
        2000ms);

    EXPECT_TRUE(result.mark_lost);
    EXPECT_EQ(check_speed_count_, 0);
}

// Tracker sees no movement but the target reports it is moving: the tracker
// is wrong and must be marked lost.
TEST_F(TrackerRecoveryMonitorTest, NoMovementConfirmedMovingMarksLost) {
    mock_is_moving_ = true;
    auto monitor = MakeMonitor();

    const auto result = TickUntil(
        *monitor, TrackingInput(0.0, 1.0),
        [](const TrackerRecoveryMonitor::UpdateResult& r) { return r.mark_lost; },
        3000ms);

    EXPECT_TRUE(result.mark_lost);
    EXPECT_GE(check_speed_count_, 1);
    EXPECT_EQ(last_check_speed_request_.operation, "gt");
    EXPECT_DOUBLE_EQ(last_check_speed_request_.speed, 0.2);
}

// Target confirms it is genuinely stationary: not lost, and the next
// confirmation call only happens after the backoff time.
TEST_F(TrackerRecoveryMonitorTest, NoMovementConfirmedStationaryBacksOff) {
    mock_is_moving_ = false;
    auto monitor = MakeMonitor();
    const auto input = TrackingInput(0.0, 1.0);

    TickUntil(
        *monitor, input,
        [this](const TrackerRecoveryMonitor::UpdateResult&) {
            return check_speed_count_ >= 1;
        },
        3000ms);
    ASSERT_EQ(check_speed_count_, 1);
    const auto first_confirm_time = std::chrono::steady_clock::now();

    const auto result = TickUntil(
        *monitor, input,
        [this](const TrackerRecoveryMonitor::UpdateResult& r) {
            return r.mark_lost || check_speed_count_ >= 2;
        },
        3000ms);

    EXPECT_FALSE(result.mark_lost);
    EXPECT_EQ(check_speed_count_, 2);
    const double elapsed =
        std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                      first_confirm_time)
            .count();
    EXPECT_GE(elapsed, 0.4);  // no_movement_backoff_time = 0.5 s, with margin
}

// While LOST, the monitor fetches ground-truth odometry for recovery.
TEST_F(TrackerRecoveryMonitorTest, LostTriggersGroundTruthRecovery) {
    mock_odom_ = ValidOdom(5.0, 6.0);
    auto monitor = MakeMonitor();

    const auto result = TickUntil(
        *monitor, LostInput(),
        [](const TrackerRecoveryMonitor::UpdateResult& r) {
            return r.recovery_odom.has_value();
        },
        3000ms);

    ASSERT_TRUE(result.recovery_odom.has_value());
    EXPECT_EQ(get_odometry_count_, 1);
    EXPECT_EQ(result.recovery_odom->header.frame_id, "map");
    EXPECT_DOUBLE_EQ(result.recovery_odom->pose.pose.position.x, 5.0);
    EXPECT_DOUBLE_EQ(result.recovery_odom->pose.pose.position.y, 6.0);
}

// Unreachable services (no such vehicle) must neither crash nor mark lost.
TEST_F(TrackerRecoveryMonitorTest, UnavailableServicesAreTolerated) {
    auto monitor = MakeMonitor("ghost");

    auto result = TickUntil(
        *monitor, TrackingInput(0.0, 1.0),
        [](const TrackerRecoveryMonitor::UpdateResult& r) { return r.mark_lost; },
        500ms);
    EXPECT_FALSE(result.mark_lost);

    result = TickUntil(
        *monitor, LostInput(),
        [](const TrackerRecoveryMonitor::UpdateResult& r) {
            return r.recovery_odom.has_value();
        },
        500ms);
    EXPECT_FALSE(result.recovery_odom.has_value());
}

// Transitioning to a state outside no_movement_check_in_states resets the
// speed window: it must fill from scratch before a confirmation is sent.
TEST_F(TrackerRecoveryMonitorTest, LeavingCheckStatesResetsSpeedWindow) {
    mock_is_moving_ = false;
    auto monitor = MakeMonitor();
    const auto stationary = TrackingInput(0.0, 1.0);

    // Half a window of no movement: not enough to confirm.
    TickFor(*monitor, stationary, 50ms);
    EXPECT_EQ(check_speed_count_, 0);

    // One tick outside the check states resets the window.
    auto out_of_scope = stationary;
    out_of_scope.state = TrackerState::NO_DETECTION;
    monitor->Update(out_of_scope);

    // Another half window: still nothing, the window restarted.
    TickFor(*monitor, stationary, 50ms);
    EXPECT_EQ(check_speed_count_, 0);

    // A full window after the reset finally confirms.
    TickUntil(
        *monitor, stationary,
        [this](const TrackerRecoveryMonitor::UpdateResult&) {
            return check_speed_count_ >= 1;
        },
        2000ms);
    EXPECT_EQ(check_speed_count_, 1);
}

// A default-constructed (all-zero) odometry response means the mission
// manager has no odometry yet: it must be rejected and retried.
TEST_F(TrackerRecoveryMonitorTest, EmptyRecoveryOdomRejectedAndRetried) {
    mock_odom_ = nav_msgs::msg::Odometry();
    auto monitor = MakeMonitor();

    auto result = TickUntil(
        *monitor, LostInput(),
        [this](const TrackerRecoveryMonitor::UpdateResult& r) {
            return r.recovery_odom.has_value() || get_odometry_count_ >= 1;
        },
        2000ms);
    EXPECT_FALSE(result.recovery_odom.has_value());
    EXPECT_GE(get_odometry_count_, 1);

    // Once valid odometry becomes available, a retry recovers.
    mock_odom_ = ValidOdom(1.0, 2.0);
    result = TickUntil(
        *monitor, LostInput(),
        [](const TrackerRecoveryMonitor::UpdateResult& r) {
            return r.recovery_odom.has_value();
        },
        3000ms);
    ASSERT_TRUE(result.recovery_odom.has_value());
    EXPECT_DOUBLE_EQ(result.recovery_odom->pose.pose.position.x, 1.0);
}

// While the tracker stays LOST, the ground-truth odometry is re-fetched at
// the backoff pace (not on every tick) to keep the published estimate
// current.
TEST_F(TrackerRecoveryMonitorTest, RecoveryRefreshIsPacedWhileLost) {
    mock_odom_ = ValidOdom(5.0, 6.0);
    auto monitor = MakeMonitor();

    auto result = TickUntil(
        *monitor, LostInput(),
        [](const TrackerRecoveryMonitor::UpdateResult& r) {
            return r.recovery_odom.has_value();
        },
        3000ms);
    ASSERT_TRUE(result.recovery_odom.has_value());
    EXPECT_EQ(get_odometry_count_, 1);
    const auto first_recovery_time = std::chrono::steady_clock::now();

    result = TickUntil(
        *monitor, LostInput(),
        [](const TrackerRecoveryMonitor::UpdateResult& r) {
            return r.recovery_odom.has_value();
        },
        3000ms);
    ASSERT_TRUE(result.recovery_odom.has_value());
    EXPECT_EQ(get_odometry_count_, 2);
    const double elapsed =
        std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                      first_recovery_time)
            .count();
    EXPECT_GE(elapsed, 0.4);  // no_movement_backoff_time = 0.5 s, with margin
}

// A disabled no-movement pathway must never trigger a confirmation service
// call or a lost verdict, even when the tracker sees no movement and the
// target reports it is moving.
TEST_F(TrackerRecoveryMonitorTest, DisabledNoMovementCheckIsInert) {
    mock_is_moving_ = true;
    RecoverySettings settings = MakeSettings();
    settings.no_movement_enabled = false;
    TrackerRecoveryMonitor monitor(host_node_.get(), "agv1", settings,
                                   host_node_->get_logger());

    const auto result = TickUntil(
        monitor, TrackingInput(0.0, 1.0),
        [](const TrackerRecoveryMonitor::UpdateResult& r) { return r.mark_lost; },
        400ms);

    EXPECT_FALSE(result.mark_lost);
    EXPECT_EQ(check_speed_count_, 0);
}

// A disabled uncertainty pathway must never mark lost, even under a
// covariance far above the threshold.
TEST_F(TrackerRecoveryMonitorTest, DisabledUncertaintyCheckIsInert) {
    RecoverySettings settings = MakeSettings();
    settings.uncertainty_enabled = false;
    TrackerRecoveryMonitor monitor(host_node_.get(), "agv1", settings,
                                   host_node_->get_logger());

    const auto result = TickUntil(
        monitor, TrackingInput(5.0, 400.0),
        [](const TrackerRecoveryMonitor::UpdateResult& r) { return r.mark_lost; },
        400ms);

    EXPECT_FALSE(result.mark_lost);
}

// Measurement starvation after sustained active tracking (target left the
// sensor field of view) marks the tracker lost without a service
// confirmation.
TEST_F(TrackerRecoveryMonitorTest, MeasurementTimeoutAfterTrackingMarksLost) {
    auto monitor = MakeMonitor();

    // Sustained genuine tracking arms the measurement-timeout check.
    TickFor(*monitor, TrackingInput(1.0, 1.0), 1200ms);

    const auto result = monitor->Update(StarvedInput());
    EXPECT_TRUE(result.mark_lost);
    EXPECT_EQ(check_speed_count_, 0);
}

// Measurement starvation without prior active tracking (target never
// acquired) must not mark lost.
TEST_F(TrackerRecoveryMonitorTest, MeasurementTimeoutWithoutTrackingIsIgnored) {
    auto monitor = MakeMonitor();

    for (int i = 0; i < 5; ++i) {
        EXPECT_FALSE(monitor->Update(StarvedInput()).mark_lost);
    }
}

// With allow_never_tracked, starvation marks lost even when the target was
// never acquired (fresh tracker in INACTIVE, filter never initialized).
TEST_F(TrackerRecoveryMonitorTest, NeverTrackedTimeoutMarksLostWhenAllowed) {
    RecoverySettings settings = MakeSettings();
    settings.timeout_allow_never_tracked = true;
    TrackerRecoveryMonitor monitor(host_node_.get(), "agv1", settings,
                                   host_node_->get_logger());

    TrackerRecoveryMonitor::TickInput input;
    input.state = TrackerState::INACTIVE;
    input.filter_initialized = false;
    input.time_since_valid_target = 1.0;
    input.target_timeout = 0.5;

    const auto result = monitor.Update(input);
    EXPECT_TRUE(result.mark_lost);
    EXPECT_EQ(check_speed_count_, 0);
}

// After timeout_max_attempts timeout-triggered recoveries without a
// sustained re-acquisition the pathway gives up; a sustained re-acquisition
// re-arms it.
TEST_F(TrackerRecoveryMonitorTest, MeasurementTimeoutGivesUpAfterMaxAttempts) {
    RecoverySettings settings = MakeSettings();
    settings.timeout_max_attempts = 1;
    TrackerRecoveryMonitor monitor(host_node_.get(), "agv1", settings,
                                   host_node_->get_logger());

    TickFor(monitor, TrackingInput(1.0, 1.0), 1200ms);
    EXPECT_TRUE(monitor.Update(StarvedInput()).mark_lost);  // Attempt 1.

    mock_odom_ = ValidOdom(1.0, 2.0);
    const auto recovery = TickUntil(
        monitor, LostInput(),
        [](const TrackerRecoveryMonitor::UpdateResult& r) {
            return r.recovery_odom.has_value();
        },
        3000ms);
    ASSERT_TRUE(recovery.recovery_odom.has_value());

    // Still starving with no re-acquisition: the cap suppresses further
    // attempts.
    for (int i = 0; i < 5; ++i) {
        EXPECT_FALSE(monitor.Update(StarvedInput()).mark_lost);
    }

    // A sustained re-acquisition clears the cap and re-arms the pathway.
    TickFor(monitor, TrackingInput(1.0, 1.0), 1200ms);
    EXPECT_TRUE(monitor.Update(StarvedInput()).mark_lost);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    rclcpp::init(argc, argv);
    int result = RUN_ALL_TESTS();
    rclcpp::shutdown();
    return result;
}
