#include <gtest/gtest.h>

#include <map>
#include <string>

#include "avt_341_nav/mission/formation_utils.h"
#include "nav_msgs/msg/odometry.hpp"

namespace {

nav_msgs::msg::Odometry MakeOdom(double x, double y)
{
    nav_msgs::msg::Odometry odom;
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    return odom;
}

geometry_msgs::msg::Point MakePoint(double x, double y, double z = 0.0)
{
    geometry_msgs::msg::Point point;
    point.x = x;
    point.y = y;
    point.z = z;
    return point;
}

}  // namespace

TEST(FindVehicleWithin, NoVehicles)
{
    EXPECT_FALSE(FindVehicleWithin(MakePoint(0.0, 0.0), {}, 5.0).has_value());
}

TEST(FindVehicleWithin, ReturnsVehicleInsideRadius)
{
    const std::map<std::string, nav_msgs::msg::Odometry> poses{
        {"FEDA", MakeOdom(10.0, 0.0)}, {"MRZR", MakeOdom(100.0, 0.0)}};
    const auto match = FindVehicleWithin(MakePoint(13.0, 4.0, 2.0), poses, 5.1);
    ASSERT_TRUE(match.has_value());
    EXPECT_EQ(*match, "FEDA");
}

TEST(FindVehicleWithin, IgnoresVehiclesOutsideRadius)
{
    const std::map<std::string, nav_msgs::msg::Odometry> poses{{"FEDA", MakeOdom(10.0, 0.0)}};
    EXPECT_FALSE(FindVehicleWithin(MakePoint(15.0, 0.0), poses, 5.0).has_value());
}

TEST(FindVehicleWithin, ReturnsClosestVehicle)
{
    const std::map<std::string, nav_msgs::msg::Odometry> poses{
        {"FEDA", MakeOdom(3.0, 0.0)}, {"MRZR2", MakeOdom(-1.0, 0.0)}};
    const auto match = FindVehicleWithin(MakePoint(0.0, 0.0), poses, 5.0);
    ASSERT_TRUE(match.has_value());
    EXPECT_EQ(*match, "MRZR2");
}

TEST(FindVehicleOnLineOfSight, ReportShortOfTheVehicleAlongTheLineOfSightMatches)
{
    const std::map<std::string, nav_msgs::msg::Odometry> poses{{"MRZR2", MakeOdom(40.0, 0.0)}};
    const auto match = FindVehicleOnLineOfSight(MakePoint(28.0, 2.0), MakePoint(0.0, 0.0), poses, 3.5, 0.35, 7.0);
    ASSERT_TRUE(match.has_value());
    EXPECT_EQ(*match, "MRZR2");
}

TEST(FindVehicleOnLineOfSight, ReportBesideTheLineOfSightDoesNotMatch)
{
    const std::map<std::string, nav_msgs::msg::Odometry> poses{{"MRZR2", MakeOdom(40.0, 0.0)}};
    EXPECT_FALSE(FindVehicleOnLineOfSight(MakePoint(38.0, 4.0), MakePoint(0.0, 0.0), poses, 3.5, 0.35, 7.0).has_value());
}

TEST(FindVehicleOnLineOfSight, RangeToleranceGrowsWithRangeAboveItsMinimum)
{
    const std::map<std::string, nav_msgs::msg::Odometry> near{{"MRZR2", MakeOdom(10.0, 0.0)}};
    EXPECT_TRUE(FindVehicleOnLineOfSight(MakePoint(16.5, 0.0), MakePoint(0.0, 0.0), near, 3.5, 0.35, 7.0).has_value());
    EXPECT_FALSE(FindVehicleOnLineOfSight(MakePoint(17.5, 0.0), MakePoint(0.0, 0.0), near, 3.5, 0.35, 7.0).has_value());
    const std::map<std::string, nav_msgs::msg::Odometry> far{{"MRZR2", MakeOdom(60.0, 0.0)}};
    EXPECT_TRUE(FindVehicleOnLineOfSight(MakePoint(80.0, 0.0), MakePoint(0.0, 0.0), far, 3.5, 0.35, 7.0).has_value());
    EXPECT_FALSE(FindVehicleOnLineOfSight(MakePoint(82.0, 0.0), MakePoint(0.0, 0.0), far, 3.5, 0.35, 7.0).has_value());
}

TEST(FindVehicleOnLineOfSight, ReportBehindTheObserverDoesNotMatch)
{
    const std::map<std::string, nav_msgs::msg::Odometry> poses{{"MRZR2", MakeOdom(5.0, 0.0)}};
    EXPECT_FALSE(FindVehicleOnLineOfSight(MakePoint(-1.0, 0.0), MakePoint(0.0, 0.0), poses, 3.5, 0.35, 7.0).has_value());
}

TEST(FindVehicleWithin, ZeroRadiusMatchesNothing)
{
    const std::map<std::string, nav_msgs::msg::Odometry> poses{{"FEDA", MakeOdom(0.0, 0.0)}};
    EXPECT_FALSE(FindVehicleWithin(MakePoint(0.0, 0.0), poses, 0.0).has_value());
}
