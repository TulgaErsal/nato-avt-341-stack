/**
 * Unit tests for StaticGridLayer
 */

#include <gtest/gtest.h>
#include <string>

#include "avt_341_nav/perception/layers/static_grid_layer.h"

using avt_341_nav::perception::StaticGridLayer;
using avt_341_nav::perception::CostmapSizeInfo;

struct ParseSizeInfoTestParams
{
    std::string label;
    std::string filename;
    CostmapSizeInfo expected;
};

// Expected filename format [prefix_]x_{x}_y_{y}_res_{res}_w_{w}_h_{h}.csv
class ParseSizeInfoFromFileTest
    : public ::testing::TestWithParam<ParseSizeInfoTestParams> {};

TEST_P(ParseSizeInfoFromFileTest, ParsesCorrectly)
{
    const auto& tc = GetParam();
    const auto info = StaticGridLayer::ParseSizeInfoFromFile(tc.filename);

    EXPECT_EQ(info.llx, tc.expected.llx);
    EXPECT_EQ(info.lly, tc.expected.lly);
    EXPECT_EQ(info.res, tc.expected.res);
    EXPECT_EQ(info.width, tc.expected.width);
    EXPECT_EQ(info.height, tc.expected.height);
}

// ---------------------------------------------------------------------------
// Test cases
// ---------------------------------------------------------------------------
INSTANTIATE_TEST_SUITE_P(
    AllCases,
    ParseSizeInfoFromFileTest,
    ::testing::Values(
        ParseSizeInfoTestParams{
            "CanonicalFilename",
            "map_x_100_y_200_res_2_w_500_h_600.csv",
            {500.0f, 600.0f, 2.0f, 100.0f, 200.0f}},
        ParseSizeInfoTestParams{
            "StripsUnixPath",
            "/home/user/data/x_10_y_20_res_1_w_100_h_200.csv",
            {100.0f, 200.0f, 1.0f, 10.0f, 20.0f}},
        ParseSizeInfoTestParams{
            "StripsWindowsPath",
            "C:\\Users\\user\\data\\x_50_y_60_res_5_w_1000_h_2000.csv",
            {1000.0f, 2000.0f, 5.0f, 50.0f, 60.0f}},
        ParseSizeInfoTestParams{
            "FloatingPointValues",
            "x_1.5_y_2.5_res_0.25_w_50.5_h_75.75.csv",
            {50.5f, 75.75f, 0.25f, 1.5f, 2.5f}},
        ParseSizeInfoTestParams{
            "NegativeCoordinates",
            "x_-100_y_-200_res_1_w_400_h_400.csv",
            {400.0f, 400.0f, 1.0f, -100.0f, -200.0f}},
        ParseSizeInfoTestParams{
            "UnrecognisedFilename",
            "random_name_without_keys.csv",
            {0.0f, 0.0f, 1.0f, 0.0f, 0.0f}},
        ParseSizeInfoTestParams{
            "EmptyString",
            "",
            {0.0f, 0.0f, 1.0f, 0.0f, 0.0f}}
    ),
    [](const ::testing::TestParamInfo<ParseSizeInfoTestParams>& info) {
        return info.param.label;
    });
