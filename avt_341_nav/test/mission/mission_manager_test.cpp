#include <gtest/gtest.h>

#include <cmath>
#include <map>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "avt_341_msgs/msg/communication.hpp"
#include "avt_341_nav/mission/goal_filtering/goal_filter_factory.hpp"
#include "avt_341_nav/mission/mission_manager.h"
#include "avt_341_nav/mission/mission_manager_parser.h"
#include "avt_341_nav/mission/task.h"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"

using avt_341_nav::mission::MissionManager;
using avt_341_nav::mission::MissionManagerParams;
using avt_341_nav::mission::Encircle;

namespace {

avt_341_msgs::msg::Communication MakeCommand(const std::string & type, const std::string & toi_regex)
{
    avt_341_msgs::msg::Communication msg;
    msg.sender_name = "MRZR";
    msg.msg_id = 3;
    msg.type = type;
    msg.receiver_name = "MRZR";
    msg.objective_name = "MP_Y";
    msg.desired_speed = 5.0;
    msg.priority_type = "QUEUE";
    msg.yaw_threshold = -1.0;
    msg.toi_regex = toi_regex;
    return msg;
}

nav_msgs::msg::Odometry MakeOdom(double x, double y)
{
    nav_msgs::msg::Odometry odom;
    odom.header.frame_id = "map";
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.orientation.w = 1.0;
    return odom;
}

nav_msgs::msg::Path MakeContact(const std::string & name, double x, double y)
{
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = name;
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.orientation.w = 1.0;
    nav_msgs::msg::Path contacts;
    contacts.header.frame_id = "map";
    contacts.poses.push_back(pose);
    return contacts;
}

}  // namespace

// toiRegexUpdate
// ------------------------------------------------------------------------------------------------

TEST(ToiRegexUpdate, NonEmptyRegexAppliesForFormMoveToAndPathFollow)
{
    for(const auto & type : {MissionMsgType::Formation, MissionMsgType::MoveTo, MissionMsgType::PathFollow}) {
        EXPECT_EQ(toiRegexUpdate(MakeCommand(type, "car")), std::optional<std::string>("car")) << type;
        EXPECT_EQ(toiRegexUpdate(MakeCommand(type, "^$")), std::optional<std::string>("^$")) << type;
    }
}

TEST(ToiRegexUpdate, EmptyRegexKeepsCurrentForEveryCommand)
{
    for(const auto & type : {MissionMsgType::Formation, MissionMsgType::MoveTo, MissionMsgType::PathFollow}) {
        EXPECT_FALSE(toiRegexUpdate(MakeCommand(type, "")).has_value()) << type;
    }
}

TEST(ToiRegexUpdate, OtherCommandsNeverApply)
{
    EXPECT_FALSE(toiRegexUpdate(MakeCommand(MissionMsgType::SetSpeed, "car")).has_value());
    EXPECT_FALSE(toiRegexUpdate(MakeCommand(MissionMsgType::Cancel, "car")).has_value());
}

// Text serialization (socket comms)
// ------------------------------------------------------------------------------------------------

TEST(SerializedToiRegex, MoveToRoundTripKeepsRegexWithCommas)
{
    const auto parsed = serializedToROSMsg(rosToSerializedMsg(MakeCommand(MissionMsgType::MoveTo, "TI_[0-9]{1,3},car")));
    EXPECT_EQ(parsed.type, MissionMsgType::MoveTo);
    EXPECT_EQ(parsed.objective_name, "MP_Y");
    EXPECT_EQ(parsed.priority_type, "QUEUE");
    EXPECT_EQ(parsed.toi_regex, "TI_[0-9]{1,3},car");
}

TEST(SerializedToiRegex, PathFollowRoundTripKeepsRegexWithCommas)
{
    const auto parsed = serializedToROSMsg(rosToSerializedMsg(MakeCommand(MissionMsgType::PathFollow, "^car$,truck")));
    EXPECT_EQ(parsed.type, MissionMsgType::PathFollow);
    EXPECT_EQ(parsed.objective_name, "MP_Y");
    EXPECT_DOUBLE_EQ(parsed.desired_speed, 5.0);
    EXPECT_EQ(parsed.priority_type, "QUEUE");
    EXPECT_EQ(parsed.toi_regex, "^car$,truck");
}

TEST(SerializedToiRegex, FormationRoundTripKeepsRegex)
{
    auto msg = MakeCommand(MissionMsgType::Formation, "car");
    msg.formation = "COLUMN";
    msg.leader_name = "MRZR";
    msg.follower1_name = "FEDA";
    msg.termination_method = "LEADER_ARRIVED";
    EXPECT_EQ(serializedToROSMsg(rosToSerializedMsg(msg)).toi_regex, "car");
}

TEST(SerializedToiRegex, CommandsWithoutRegexLeaveTrackerUnchanged)
{
    for(const auto & text : {std::string("MRZR,0,MOVETO,MRZR,MP_Y,QUEUE"),
                             std::string("MRZR,0,MOVETO,MRZR,MP_Y,0,0,0,-1,QUEUE"),
                             std::string("MRZR,0,MOVETO,MRZR,MP_Y,0,0,0,-1,QUEUE,"),
                             std::string("MRZR,0,PATH_FOLLOW,MRZR,PATH_A,5,QUEUE"),
                             std::string("MRZR,0,PATH_FOLLOW,MRZR,PATH_A,5,QUEUE,"),
                             std::string("MRZR,0,FORM,COLUMN,MRZR,FEDA,,,MP_AD,5,10,10,0,0,0,-1,LEADER_ARRIVED,QUEUE"),
                             std::string("MRZR,0,FORM,COLUMN,MRZR,FEDA,,,MP_AD,5,10,10,0,0,0,-1,LEADER_ARRIVED,QUEUE,")}) {
        const auto parsed = serializedToROSMsg(text);
        EXPECT_EQ(parsed.toi_regex, "") << text;
        EXPECT_FALSE(toiRegexUpdate(parsed).has_value()) << text;
    }
}

TEST(SerializedToiRegex, TrailingLineEndingsAreStripped)
{
    for(const auto & ending : {std::string("\n"), std::string("\r\n"), std::string(" \r\n")}) {
        EXPECT_EQ(serializedToROSMsg("MRZR,0,MOVETO,MRZR,MP_Y,0,0,0,-1,QUEUE,car" + ending).toi_regex, "car");
        EXPECT_EQ(serializedToROSMsg("MRZR,0,MOVETO,MRZR,MP_Y,0,0,0,-1,QUEUE" + ending).priority_type, "QUEUE");
        const auto no_regex = serializedToROSMsg("MRZR,0,MOVETO,MRZR,MP_Y,0,0,0,-1,QUEUE," + ending);
        EXPECT_EQ(no_regex.toi_regex, "");
        EXPECT_FALSE(toiRegexUpdate(no_regex).has_value());
        EXPECT_EQ(serializedToROSMsg("MRZR,0,PATH_FOLLOW,MRZR,PATH_A,5,QUEUE,^TI_" + ending).toi_regex, "^TI_");
    }
    EXPECT_EQ(serializedToROSMsg("\r\n").type, "");
}

TEST(SerializedMessages, MalformedFieldCountsParseToEmptyMessage)
{
    for(const auto & text : {std::string("MRZR,0,MOVETO,MRZR,MP_Y,0,0"),
                             std::string("MRZR,0,MOVETO,MRZR,MP_Y,0,0,0,-1"),
                             std::string("MRZR,0,MOVETO,MRZR"),
                             std::string("MRZR,0,FORM,COLUMN,MRZR,FEDA,,,MP_AD"),
                             std::string("MRZR,0,FORM,COLUMN,MRZR,FEDA,,,MP_AD,5,10,10,0,0,0,-1"),
                             std::string("MRZR,0,PATH_FOLLOW,MRZR,PATH_A,5"),
                             std::string("MRZR,0,WAIT_UNTIL_COMPLETE,MRZR,FEDA"),
                             std::string("MRZR,0,SET_SPEED,MRZR"),
                             std::string("MRZR,0,CANCEL_ALL")}) {
        EXPECT_EQ(serializedToROSMsg(text).type, "") << text;
    }
}

TEST(SerializedMessages, ValidShortFormatsStillParse)
{
    EXPECT_EQ(serializedToROSMsg("MRZR,0,MOVETO,MRZR,MP_Y,QUEUE").objective_name, "MP_Y");
    EXPECT_EQ(serializedToROSMsg("MRZR,0,FORM,COLUMN,MRZR,FEDA,,,MP_AD,5").objective_name, "MP_AD");
    EXPECT_EQ(serializedToROSMsg("MRZR,0,FORM,COLUMN,MRZR,FEDA,,,MP_AD,5,QUEUE").priority_type, "QUEUE");
    EXPECT_DOUBLE_EQ(serializedToROSMsg("MRZR,0,SET_SPEED,MRZR,4.5").desired_speed, 4.5);
    EXPECT_EQ(serializedToROSMsg("MRZR,-1,ARRIVE,TASK_0").objective_name, "TASK_0");
}

// Team vehicle contact filter
// ------------------------------------------------------------------------------------------------

class MissionManagerContactsTest : public ::testing::Test {
  protected:
    static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
    static void TearDownTestSuite() { rclcpp::shutdown(); }

    void SetUp() override
    {
        node_ = std::make_shared<rclcpp::Node>("mission_manager_contacts_test");
        params_.toi.contact_trigger_delay = 0.0;
        team_ = {{"MRZR", MakeOdom(0.0, 0.0)}, {"FEDA", MakeOdom(50.0, 0.0)}};
    }

    std::unique_ptr<MissionManager> MakeManager()
    {
        auto goal_filter = avt_341_nav::mission::create_goal_filter("MRZR", "none", node_, params_.fgf_obs_avoid);
        return std::make_unique<MissionManager>(params_, "MRZR", node_, goal_filter);
    }

    rclcpp::Node::SharedPtr node_;
    MissionManagerParams params_;
    std::map<std::string, nav_msgs::msg::Odometry> team_;
};

TEST_F(MissionManagerContactsTest, ContactAwayFromTeamCreatesInvestigationTasks)
{
    auto manager = MakeManager();
    manager->handleContacts(MakeContact("car", 50.0, 30.0), team_);
    ASSERT_NE(manager->currentTask(), nullptr);
    EXPECT_NE(manager->currentTask()->description().find("CONTACT car"), std::string::npos);
}

TEST_F(MissionManagerContactsTest, ContactAtTeamVehicleIsIgnored)
{
    auto manager = MakeManager();
    manager->handleContacts(MakeContact("car", 52.0, 3.0), team_);
    EXPECT_EQ(manager->currentTask(), nullptr);
}

TEST_F(MissionManagerContactsTest, ContactAtEgoVehicleIsIgnored)
{
    auto manager = MakeManager();
    manager->handleContacts(MakeContact("car", 1.0, -1.0), team_);
    EXPECT_EQ(manager->currentTask(), nullptr);
}

TEST_F(MissionManagerContactsTest, ZeroRadiusDisablesFilter)
{
    params_.toi.team_vehicle_exclusion_radius = 0.0;
    auto manager = MakeManager();
    manager->handleContacts(MakeContact("car", 52.0, 3.0), team_);
    EXPECT_NE(manager->currentTask(), nullptr);
}

TEST_F(MissionManagerContactsTest, IgnoredContactDoesNotBlockLaterSighting)
{
    auto manager = MakeManager();
    manager->handleContacts(MakeContact("car", 50.0, 2.0), team_);
    EXPECT_EQ(manager->currentTask(), nullptr);
    manager->handleContacts(MakeContact("car", 50.0, 30.0), team_);
    EXPECT_NE(manager->currentTask(), nullptr);
}

// Encircle re-planning on contact updates
// ------------------------------------------------------------------------------------------------

namespace {

geometry_msgs::msg::PoseStamped MakePose(double x, double y)
{
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.orientation.w = 1.0;
    return pose;
}

double BearingDeg(const geometry_msgs::msg::PoseStamped & pose, double cx, double cy)
{
    return std::atan2(pose.pose.position.y - cy, pose.pose.position.x - cx) * 180.0 / M_PI;
}

double DistanceTo(const geometry_msgs::msg::PoseStamped & pose, double cx, double cy)
{
    return std::hypot(pose.pose.position.x - cx, pose.pose.position.y - cy);
}

}  // namespace

class EncircleReplanTest : public MissionManagerContactsTest {
  protected:
    void SetUp() override
    {
        MissionManagerContactsTest::SetUp();
        params_.toi.encircle_radius = 10.0;
        params_.toi.encircle_degrees = 180.0;
        params_.toi.encircle_cw = true;
        params_.toi.encircle_replan_distance = 2.0;
    }

    std::unique_ptr<Encircle> StartEncircle(MissionManager & manager, double vx, double vy)
    {
        manager.odometry = MakeOdom(vx, vy);
        auto encircle = std::make_unique<Encircle>(&manager, "MRZR", 9999, MakePose(0.0, 0.0), params_.toi);
        encircle->init();
        return encircle;
    }
};

TEST_F(EncircleReplanTest, QueuedEncircleOnlyTakesTheNewTarget)
{
    auto manager = MakeManager();
    Encircle encircle(manager.get(), "MRZR", 9999, MakePose(0.0, 0.0), params_.toi);
    EXPECT_FALSE(encircle.updateTarget(MakePose(30.0, 0.0)));
    EXPECT_DOUBLE_EQ(encircle.target().pose.position.x, 30.0);
    EXPECT_TRUE(encircle.circlePath().poses.empty());
}

TEST_F(EncircleReplanTest, RunningEncircleReplansTheRemainingArcAroundTheMovedTarget)
{
    auto manager = MakeManager();
    auto encircle = StartEncircle(*manager, 20.0, 0.0);
    ASSERT_EQ(encircle->circlePath().poses.size(), 180u);
    EXPECT_NEAR(BearingDeg(encircle->circlePath().poses.back(), 0.0, 0.0), -179.0, 1e-6);

    manager->odometry = MakeOdom(0.0, -10.0);
    ASSERT_TRUE(encircle->updateTarget(MakePose(0.0, 5.0)));
    EXPECT_DOUBLE_EQ(encircle->target().pose.position.y, 5.0);

    const auto & path = encircle->circlePath().poses;
    ASSERT_FALSE(path.empty());
    for (const auto & pose : path) {
        EXPECT_NEAR(DistanceTo(pose, 0.0, 5.0), 10.0, 1e-6);
    }
    EXPECT_NEAR(BearingDeg(path.front(), 0.0, 5.0), -90.0, 1e-6);
    EXPECT_NEAR(BearingDeg(path.back(), 0.0, 5.0), -179.0, 1.0);
    EXPECT_NEAR(static_cast<double>(path.size()), 90.0, 1.0);
}

TEST_F(EncircleReplanTest, SmallContactUpdatesKeepTheCircle)
{
    auto manager = MakeManager();
    auto encircle = StartEncircle(*manager, 20.0, 0.0);
    const auto before = encircle->circlePath().poses;
    EXPECT_FALSE(encircle->updateTarget(MakePose(1.0, 1.0)));
    EXPECT_DOUBLE_EQ(encircle->target().pose.position.x, 0.0);
    ASSERT_EQ(encircle->circlePath().poses.size(), before.size());
    EXPECT_DOUBLE_EQ(encircle->circlePath().poses.back().pose.position.y, before.back().pose.position.y);
}

TEST_F(EncircleReplanTest, ZeroReplanDistanceKeepsTheCircle)
{
    params_.toi.encircle_replan_distance = 0.0;
    auto manager = MakeManager();
    auto encircle = StartEncircle(*manager, 20.0, 0.0);
    EXPECT_FALSE(encircle->updateTarget(MakePose(0.0, 30.0)));
    EXPECT_NEAR(DistanceTo(encircle->circlePath().poses.back(), 0.0, 0.0), 10.0, 1e-6);
}

TEST_F(EncircleReplanTest, VehiclePastTheEndOfTheArcGetsOnlyTheEndPoint)
{
    auto manager = MakeManager();
    auto encircle = StartEncircle(*manager, 20.0, 0.0);
    const double past_end = (-179.0 - 10.0) * M_PI / 180.0;
    manager->odometry = MakeOdom(10.0 * std::cos(past_end), 5.0 + 10.0 * std::sin(past_end));
    ASSERT_TRUE(encircle->updateTarget(MakePose(0.0, 5.0)));
    ASSERT_EQ(encircle->circlePath().poses.size(), 1u);
    EXPECT_NEAR(BearingDeg(encircle->circlePath().poses.front(), 0.0, 5.0), -179.0, 1e-6);
}
