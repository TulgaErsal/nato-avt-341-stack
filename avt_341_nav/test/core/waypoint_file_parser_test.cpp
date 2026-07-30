#include <gtest/gtest.h>

#include <fstream>
#include <stdexcept>
#include <string>

#include "avt_341_nav/core/waypoint_file_parser.hpp"

using avt_341_nav::core::WaypointFileParser;

namespace
{

std::string WriteTempFile(const std::string & name, const std::string & content)
{
    const std::string path = ::testing::TempDir() + name;
    std::ofstream file(path);
    file << content;
    return path;
}

}

TEST(WaypointFileParserTest, ParsesPairedLists)
{
    const auto path = WriteTempFile(
        "waypoints.yaml",
        "waypoints_x: [ -50.0, 0.0, 50.0  ]\n"
        "waypoints_y: [ 0.0, 0.0, 0.0 ]\n");

    const auto waypoints = WaypointFileParser::Parse(path);

    EXPECT_EQ(waypoints.header.frame_id, "map");
    ASSERT_EQ(waypoints.poses.size(), 3u);
    EXPECT_DOUBLE_EQ(waypoints.poses[0].pose.position.x, -50.0);
    EXPECT_DOUBLE_EQ(waypoints.poses[0].pose.position.y, 0.0);
    EXPECT_DOUBLE_EQ(waypoints.poses[2].pose.position.x, 50.0);
    EXPECT_DOUBLE_EQ(waypoints.poses[2].pose.position.y, 0.0);
}

TEST(WaypointFileParserTest, ParsesFrameTrailingCommasAndComments)
{
    const auto path = WriteTempFile(
        "framed_waypoints.yaml",
        "# comment line\n"
        "frame: epsg_6495\n"
        "waypoints_x: [ 3.0, 4.0, ]  # trailing comma\n"
        "waypoints_y: [ 1.0, 2.0, ]\n");

    const auto waypoints = WaypointFileParser::Parse(path);

    EXPECT_EQ(waypoints.header.frame_id, "epsg_6495");
    ASSERT_EQ(waypoints.poses.size(), 2u);
    EXPECT_EQ(waypoints.poses[0].header.frame_id, "epsg_6495");
    EXPECT_DOUBLE_EQ(waypoints.poses[1].pose.position.x, 4.0);
    EXPECT_DOUBLE_EQ(waypoints.poses[1].pose.position.y, 2.0);
}

TEST(WaypointFileParserTest, MissingKeysYieldEmptyPath)
{
    const auto path = WriteTempFile("empty.yaml", "unrelated_key: 1.0\n");

    const auto waypoints = WaypointFileParser::Parse(path);

    EXPECT_EQ(waypoints.header.frame_id, "map");
    EXPECT_TRUE(waypoints.poses.empty());
}

TEST(WaypointFileParserTest, ThrowsOnMissingFile)
{
    EXPECT_THROW(
        WaypointFileParser::Parse(::testing::TempDir() + "does_not_exist.yaml"),
        std::runtime_error);
}

TEST(WaypointFileParserTest, ThrowsOnInvalidNumber)
{
    const auto path = WriteTempFile(
        "bad_number.yaml", "waypoints_x: [ 1.0, abc ]\nwaypoints_y: [ 0.0 ]\n");

    EXPECT_THROW(WaypointFileParser::Parse(path), std::runtime_error);
}

TEST(WaypointFileParserTest, ThrowsOnMissingBrackets)
{
    const auto path = WriteTempFile(
        "bad_list.yaml", "waypoints_x: 1.0\nwaypoints_y: [ 0.0 ]\n");

    EXPECT_THROW(WaypointFileParser::Parse(path), std::runtime_error);
}

TEST(WaypointFileParserTest, ThrowsOnSizeMismatch)
{
    const auto path = WriteTempFile(
        "mismatch.yaml", "waypoints_x: [  ]\nwaypoints_y: [ 1.0, 2.0 ]\n");

    EXPECT_THROW(WaypointFileParser::Parse(path), std::runtime_error);
}
