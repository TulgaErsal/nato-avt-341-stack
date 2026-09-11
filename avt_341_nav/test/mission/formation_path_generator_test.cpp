#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "avt_341_nav/core/eigen_dto_conversion.hpp"
#include "avt_341_nav/core/ros_msg_utils.hpp"
#include "avt_341_nav/mission/formation_path_generator.h"
#include "nav_msgs/msg/odometry.hpp"

using avt_341_nav::mission::FollowerStatus;
using avt_341_nav::mission::FormationParameters;
using avt_341_nav::mission::FormationPathGenerator;

namespace {

constexpr int32_t LEADER_STAMP_SEC = 42;

nav_msgs::msg::Odometry MakeOdom(double x, double y, double yaw)
{
    nav_msgs::msg::Odometry odom;
    odom.header.frame_id = "odom";  // deliberately not "map": the generator must not copy it
    odom.header.stamp.sec = LEADER_STAMP_SEC;
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.orientation = avt_341_nav::core::YawToQuaternionMsg(yaw);
    return odom;
}

FollowerStatus MakeStatus(double x_offset, double y_offset)
{
    FollowerStatus status;
    status.use_leader = true;
    status.x_offset = x_offset;
    status.y_offset = y_offset;
    return status;
}

double YawAt(const FormationPathGenerator& generator, size_t index)
{
    return avt_341_nav::core::GetHeadingFromOrientation(
        generator.GetPath().poses[index].pose.orientation);
}

/// Collect the yaw of every pose currently on the generated path.
std::vector<double> Yaws(const FormationPathGenerator& generator)
{
    std::vector<double> yaws;
    yaws.reserve(generator.GetPath().poses.size());
    for (size_t i = 0; i < generator.GetPath().poses.size(); i++) {
        yaws.push_back(YawAt(generator, i));
    }
    return yaws;
}

/// Drive the leader along +x from (start_x, y) for num_steps points spaced step_dist apart.
void DriveEast(FormationPathGenerator& generator,
               const FollowerStatus& status,
               double start_x, double y, int num_steps, double step_dist,
               const nav_msgs::msg::Odometry& ego = nav_msgs::msg::Odometry())
{
    for (int i = 0; i < num_steps; i++) {
        generator.Update(MakeOdom(start_x + i * step_dist, y, 0.0), ego, status);
    }
}

/// Drive the leader along +y from (x, start_y) for num_steps points spaced step_dist apart.
void DriveNorth(FormationPathGenerator& generator,
                const FollowerStatus& status,
                double x, double start_y, int num_steps, double step_dist,
                const nav_msgs::msg::Odometry& ego = nav_msgs::msg::Odometry())
{
    for (int i = 0; i < num_steps; i++) {
        generator.Update(MakeOdom(x, start_y + i * step_dist, M_PI_2), ego, status);
    }
}

/// Owns the parameters, because FormationPathGenerator stores them by const reference.
class FormationPathGeneratorTest : public ::testing::Test {
protected:
    FormationParameters params;

    void SetUp() override
    {
        // Isolate the heading assertions from path trimming and from the x-offset promotion
        // logic; the tests that care about those enable them explicitly.
        params.prune_global_path = false;
        params.x_offset_on_path = false;
    }
};

class BreadcrumbFormationPathGeneratorTest : public FormationPathGeneratorTest {
protected:
    void SetUp() override
    {
        FormationPathGeneratorTest::SetUp();
        params.use_breadcrumbs = true;
        params.x_offset_on_path = true;
        params.global_path_points_dist = 1.0;
    }
};

}  // namespace

// Regression guard for the identity-quaternion bug: the seed pose (which has no predecessor on the
// path) must still carry the leader's heading.
TEST_F(FormationPathGeneratorTest, FirstPoseCarriesLeaderHeading)
{
    FormationPathGenerator generator(params);
    generator.Update(MakeOdom(0.0, 0.0, 0.7), nav_msgs::msg::Odometry(), MakeStatus(0.0, 0.0));

    ASSERT_EQ(generator.GetPath().poses.size(), 1u);
    EXPECT_NEAR(YawAt(generator, 0), 0.7, 1e-5);
}

// The tangent of a laterally offset path is parallel to the source path's tangent, so a y-offset
// must move the breadcrumbs without rotating them.
TEST_F(FormationPathGeneratorTest, HeadingIsIndependentOfLateralOffset)
{
    FormationPathGenerator centered(params);
    FormationPathGenerator offset(params);

    DriveEast(centered, MakeStatus(0.0, 0.0), 0.0, 0.0, 6, 1.5);
    DriveNorth(centered, MakeStatus(0.0, 0.0), 7.5, 1.5, 5, 1.5);
    DriveEast(offset, MakeStatus(0.0, 5.0), 0.0, 0.0, 6, 1.5);
    DriveNorth(offset, MakeStatus(0.0, 5.0), 7.5, 1.5, 5, 1.5);

    const auto& centered_poses = centered.GetPath().poses;
    const auto& offset_poses = offset.GetPath().poses;
    ASSERT_EQ(centered_poses.size(), offset_poses.size());
    ASSERT_GT(centered_poses.size(), 1u);

    // Positions differ (the offset was applied) but the headings match elementwise.
    EXPECT_GT(std::abs(centered_poses[0].pose.position.y - offset_poses[0].pose.position.y), 1.0);
    const auto centered_yaws = Yaws(centered);
    const auto offset_yaws = Yaws(offset);
    for (size_t i = 0; i < centered_yaws.size(); i++) {
        EXPECT_NEAR(centered_yaws[i], offset_yaws[i], 1e-5) << "pose index " << i;
    }
}

// Each breadcrumb records the leader's heading at the moment it was laid down, so earlier poses keep
// the old heading after the leader turns.
TEST_F(FormationPathGeneratorTest, HeadingsAreHistoricalNotGlobal)
{
    FormationPathGenerator generator(params);
    const auto status = MakeStatus(0.0, 0.0);

    DriveEast(generator, status, 0.0, 0.0, 5, 1.5);
    DriveNorth(generator, status, 6.0, 1.5, 5, 1.5);

    ASSERT_GT(generator.GetPath().poses.size(), 2u);
    EXPECT_NEAR(YawAt(generator, 0), 0.0, 1e-5);
    EXPECT_NEAR(YawAt(generator, generator.GetPath().poses.size() - 1), M_PI_2, 1e-5);
}

// With use_tangent_heading the heading comes from the leader's motion, not its reported orientation.
TEST_F(FormationPathGeneratorTest, TangentHeadingModeUsesMotionDirection)
{
    params.use_tangent_heading = true;
    FormationPathGenerator generator(params);
    const auto status = MakeStatus(0.0, 0.0);

    // Orientation says the leader faces west while it actually travels east.
    for (int i = 0; i < 6; i++) {
        generator.Update(MakeOdom(i * 1.5, 0.0, M_PI), nav_msgs::msg::Odometry(), status);
    }

    ASSERT_GT(generator.GetPath().poses.size(), 1u);
    // The tangent has not latched on the very first call, so the seed pose falls back to the
    // odometry yaw. Every later pose uses the measured direction of travel.
    EXPECT_NEAR(std::abs(YawAt(generator, 0)), M_PI, 1e-5);
    EXPECT_NEAR(YawAt(generator, generator.GetPath().poses.size() - 1), 0.0, 1e-5);
}

// Poses promoted out of leader_path_history_ into the published path must keep the heading they were
// created with. The leader drives north first so the expected heading is not zero, which the
// identity-quaternion default would otherwise satisfy by accident.
TEST_F(FormationPathGeneratorTest, XOffsetOnPathPromotionPreservesHeading)
{
    params.x_offset_on_path = true;
    FormationPathGenerator generator(params);
    const auto status = MakeStatus(-3.0, 0.0);

    DriveNorth(generator, status, 0.0, 0.0, 8, 1.5);
    DriveEast(generator, status, 1.5, 10.5, 8, 1.5);

    // The seed pose is pushed onto desired_global_path_ directly; everything after it arrives via
    // the history promotion, which lags the leader by |x_offset| of path length.
    ASSERT_GT(generator.GetPath().poses.size(), 1u);
    EXPECT_NEAR(YawAt(generator, 0), M_PI_2, 1e-5);
    for (const double yaw : Yaws(generator)) {
        EXPECT_TRUE(std::abs(yaw) < 1e-5 || std::abs(yaw - M_PI_2) < 1e-5)
            << "unexpected promoted heading " << yaw;
    }
    // The turn happened long enough ago that east-heading poses have been promoted too.
    EXPECT_NEAR(YawAt(generator, generator.GetPath().poses.size() - 1), 0.0, 1e-5);
}

// Pruning erases a prefix; it must not rewrite the headings of the surviving poses.
TEST_F(FormationPathGeneratorTest, PruneDoesNotAlterHeadings)
{
    params.prune_global_path = true;
    FormationPathGenerator generator(params);
    const auto status = MakeStatus(0.0, 0.0);

    // Ego sits well down the path so the front of the path is trimmed.
    const auto ego = MakeOdom(0.0, 9.0, M_PI_2);
    DriveNorth(generator, status, 0.0, 0.0, 10, 1.5, ego);

    ASSERT_FALSE(generator.GetPath().poses.empty());
    EXPECT_LT(generator.GetPath().poses.size(), 10u) << "expected the path front to be pruned";
    for (const double yaw : Yaws(generator)) {
        EXPECT_NEAR(yaw, M_PI_2, 1e-5);
    }
}

// The generator has no clock, so poses take the leader-odometry stamp and an explicit world frame
// (leader_odom.header.frame_id is unreliable -- see mission_manager_node).
TEST_F(FormationPathGeneratorTest, PosesCarryMapFrameAndLeaderStamp)
{
    FormationPathGenerator generator(params);
    generator.Update(MakeOdom(0.0, 0.0, 0.0), nav_msgs::msg::Odometry(), MakeStatus(0.0, 0.0));

    ASSERT_FALSE(generator.GetPath().poses.empty());
    EXPECT_EQ(generator.GetPath().header.frame_id, "map");
    EXPECT_EQ(generator.GetPath().poses.back().header.frame_id, "map");
    EXPECT_EQ(generator.GetPath().poses.back().header.stamp.sec, LEADER_STAMP_SEC);
}

TEST_F(FormationPathGeneratorTest, ResetClearsPathAndReseedsHeading)
{
    FormationPathGenerator generator(params);
    const auto status = MakeStatus(0.0, 0.0);

    DriveEast(generator, status, 0.0, 0.0, 5, 1.5);
    ASSERT_GT(generator.GetPath().poses.size(), 1u);

    generator.Reset();
    EXPECT_TRUE(generator.GetPath().poses.empty());

    generator.Update(MakeOdom(50.0, 50.0, -1.2), nav_msgs::msg::Odometry(), status);
    ASSERT_EQ(generator.GetPath().poses.size(), 1u);
    EXPECT_NEAR(YawAt(generator, 0), -1.2, 1e-5);
}

TEST_F(BreadcrumbFormationPathGeneratorTest, StartupUsesFirstHeadingAndPreservesLateralOffset)
{
    params.use_tangent_heading = true;
    FormationPathGenerator generator(params);
    const auto leader = MakeOdom(100.0, 200.0, M_PI_2);
    const auto status = MakeStatus(-10.0, 2.0);

    // Repeated stationary updates must not expose the rest of the seeded history.
    for (int i = 0; i < 3; ++i) {
        generator.Update(leader, nav_msgs::msg::Odometry(), status);
        ASSERT_EQ(generator.GetPath().poses.size(), 1u);
        const auto& pose = generator.GetPath().poses.front();
        EXPECT_NEAR(pose.pose.position.x, 102.0, 1e-5);
        EXPECT_NEAR(pose.pose.position.y, 190.0, 1e-5);
        EXPECT_NEAR(YawAt(generator, 0), M_PI_2, 1e-5);
        EXPECT_EQ(pose.header.frame_id, "map");
        EXPECT_EQ(pose.header.stamp.sec, LEADER_STAMP_SEC);
    }
}

TEST_F(BreadcrumbFormationPathGeneratorTest, PartialHistoryTransitionsAtExactOffsetWithoutDuplicates)
{
    FormationPathGenerator generator(params);
    const auto status = MakeStatus(-10.0, 3.0);

    // At 0, 5, and 10 m of real history, the target is respectively 10, 5,
    // and 0 m behind the start. Continue beyond that into entirely real history.
    for (int distance = 0; distance <= 15; ++distance) {
        generator.Update(MakeOdom(100.0 + distance, 20.0, 0.0), nav_msgs::msg::Odometry(), status);
        const auto& poses = generator.GetPath().poses;
        ASSERT_EQ(poses.size(), static_cast<size_t>(distance + 1));
        EXPECT_NEAR(poses.back().pose.position.x, 90.0 + distance, 1e-6);
        for (size_t i = 0; i < poses.size(); ++i) {
            EXPECT_NEAR(poses[i].pose.position.x, 90.0 + i, 1e-6);
            EXPECT_NEAR(poses[i].pose.position.y, 17.0, 1e-6);
        }
    }
}

TEST_F(BreadcrumbFormationPathGeneratorTest, ZeroAndFractionalOffsetsRespectBreadcrumbResolution)
{
    for (double lag : {0.0, 0.25, 2.5}) {
        SCOPED_TRACE(lag);
        FormationPathGenerator generator(params);
        const auto status = MakeStatus(-lag, 0.0);
        for (int distance = 0; distance <= 8; ++distance) {
            generator.Update(MakeOdom(distance, 0.0, 0.0), nav_msgs::msg::Odometry(), status);
            const auto& poses = generator.GetPath().poses;
            ASSERT_FALSE(poses.empty());
            const double actual_lag = distance - poses.back().pose.position.x;
            EXPECT_GE(actual_lag, lag - 1e-6);
            EXPECT_LT(actual_lag, lag + params.global_path_points_dist + 1e-6);
            EXPECT_NEAR(poses.front().pose.position.x, -lag, 1e-6);
            for (size_t i = 1; i < poses.size(); ++i) {
                EXPECT_GT(poses[i].pose.position.x, poses[i-1].pose.position.x);
            }
        }
    }
}

TEST_F(BreadcrumbFormationPathGeneratorTest, LeaderTurnDoesNotRotateSyntheticHistory)
{
    FormationPathGenerator generator(params);
    const auto status = MakeStatus(-4.0, 0.0);
    generator.Update(MakeOdom(0.0, 0.0, 0.0), nav_msgs::msg::Odometry(), status);

    for (int distance = 1; distance <= 4; ++distance) {
        generator.Update(MakeOdom(0.0, distance, M_PI_2), nav_msgs::msg::Odometry(), status);
        const auto& poses = generator.GetPath().poses;
        EXPECT_NEAR(poses.back().pose.position.x, -4.0 + distance, 1e-6);
        EXPECT_NEAR(poses.back().pose.position.y, 0.0, 1e-6);
        EXPECT_NEAR(YawAt(generator, poses.size() - 1), 0.0, 1e-6);
    }
    generator.Update(MakeOdom(0.0, 5.0, M_PI_2), nav_msgs::msg::Odometry(), status);
    const auto& poses = generator.GetPath().poses;
    EXPECT_NEAR(poses.back().pose.position.x, 0.0, 1e-6);
    EXPECT_NEAR(poses.back().pose.position.y, 1.0, 1e-6);
    EXPECT_NEAR(YawAt(generator, poses.size() - 1), M_PI_2, 1e-5);
}

TEST_F(BreadcrumbFormationPathGeneratorTest, DecimalSpacingDoesNotDuplicateLeaderStart)
{
    params.global_path_points_dist = 0.1;
    FormationPathGenerator generator(params);
    const auto status = MakeStatus(-1.0, 0.0);
    generator.Update(MakeOdom(0.0, 0.0, 0.0), nav_msgs::msg::Odometry(), status);
    generator.Update(MakeOdom(1.0, 0.0, 0.0), nav_msgs::msg::Odometry(), status);
    const auto& poses = generator.GetPath().poses;
    ASSERT_EQ(poses.size(), 11u);
    for (size_t i = 0; i < poses.size(); ++i) {
        EXPECT_NEAR(poses[i].pose.position.x, -1.0 + i * 0.1, 1e-6);
    }
}

TEST_F(BreadcrumbFormationPathGeneratorTest, PruningAndResetPreserveLagAndReseedHeading)
{
    params.prune_global_path = true;
    params.use_tangent_heading = true;
    FormationPathGenerator generator(params);
    const auto status = MakeStatus(-4.0, 2.0);
    for (int distance = 0; distance <= 8; ++distance) {
        generator.Update(MakeOdom(distance, 0.0, 0.0), MakeOdom(distance - 4.0, -2.0, 0.0), status);
        ASSERT_EQ(generator.GetPath().poses.size(), 1u);
        EXPECT_NEAR(generator.GetPath().poses.back().pose.position.x, distance - 4.0, 1e-6);
    }

    generator.Reset();
    EXPECT_TRUE(generator.GetPath().poses.empty());
    generator.Update(MakeOdom(50.0, 50.0, -M_PI_2), MakeOdom(48.0, 54.0, -M_PI_2), status);
    ASSERT_EQ(generator.GetPath().poses.size(), 1u);
    EXPECT_NEAR(generator.GetPath().poses.back().pose.position.x, 48.0, 1e-5);
    EXPECT_NEAR(generator.GetPath().poses.back().pose.position.y, 54.0, 1e-5);
    EXPECT_NEAR(YawAt(generator, 0), -M_PI_2, 1e-5);
    generator.Update(MakeOdom(50.0, 49.0, -M_PI_2), MakeOdom(48.0, 53.0, -M_PI_2), status);
    ASSERT_EQ(generator.GetPath().poses.size(), 1u);
    EXPECT_NEAR(generator.GetPath().poses.back().pose.position.y, 53.0, 1e-5);
}

TEST_F(BreadcrumbFormationPathGeneratorTest, PoseRelativeOffsetDoesNotSeedHistory)
{
    params.x_offset_on_path = false;
    FormationPathGenerator generator(params);
    generator.Update(MakeOdom(100.0, 200.0, M_PI_2), nav_msgs::msg::Odometry(), MakeStatus(-10.0, 2.0));
    ASSERT_EQ(generator.GetPath().poses.size(), 1u);
    EXPECT_NEAR(generator.GetPath().poses.back().pose.position.x, 102.0, 1e-5);
    EXPECT_NEAR(generator.GetPath().poses.back().pose.position.y, 190.0, 1e-5);
}

TEST_F(BreadcrumbFormationPathGeneratorTest, GlobalPlannerModePreservesLagThroughStartup)
{
    params.use_breadcrumbs = false;
    FormationPathGenerator generator(params);
    const auto status = MakeStatus(-10.0, 2.0);
    // Follow publishes the last pose as a goal in this mode. It must retain
    // the offset during startup and after enough real history has accumulated.
    for (int distance = 0; distance <= 15; ++distance) {
        generator.Update(MakeOdom(100.0, 200.0 + distance, M_PI_2), nav_msgs::msg::Odometry(), status);
        ASSERT_EQ(generator.GetPath().poses.size(), static_cast<size_t>(distance + 1));
        EXPECT_NEAR(generator.GetPath().poses.back().pose.position.x, 102.0, 1e-5);
        EXPECT_NEAR(generator.GetPath().poses.back().pose.position.y, 190.0 + distance, 1e-5);
    }
}
