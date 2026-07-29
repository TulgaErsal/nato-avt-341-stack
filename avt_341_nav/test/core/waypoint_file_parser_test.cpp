#include <gtest/gtest.h>

#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

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

    EXPECT_EQ(waypoints.x, (std::vector<double>{-50.0, 0.0, 50.0}));
    EXPECT_EQ(waypoints.y, (std::vector<double>{0.0, 0.0, 0.0}));
}

TEST(WaypointFileParserTest, ToleratesEmptyListsTrailingCommasAndComments)
{
    const auto path = WriteTempFile(
        "no_waypoints.yaml",
        "# comment line\n"
        "waypoints_x: [  ]\n"
        "waypoints_y: [ 1.0, 2.0, ]  # trailing comma\n");

    const auto waypoints = WaypointFileParser::Parse(path);

    EXPECT_TRUE(waypoints.x.empty());
    EXPECT_EQ(waypoints.y, (std::vector<double>{1.0, 2.0}));
}

TEST(WaypointFileParserTest, MissingKeysYieldEmptyLists)
{
    const auto path = WriteTempFile("empty.yaml", "unrelated_key: 1.0\n");

    const auto waypoints = WaypointFileParser::Parse(path);

    EXPECT_TRUE(waypoints.x.empty());
    EXPECT_TRUE(waypoints.y.empty());
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
