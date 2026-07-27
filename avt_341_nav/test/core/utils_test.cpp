#include <gtest/gtest.h>
#include <cmath>

#include "avt_341_nav/avt_341_utils.h"

using avt_341_nav::utils::DiffAngle;

struct DiffAngleTestParams
{
    std::string label;
    double a;
    double b;
    double expected;
};

class DiffAngleTest
    : public ::testing::TestWithParam<DiffAngleTestParams> {};

TEST_P(DiffAngleTest, ReturnsCorrectDifference)
{
    const auto& tc = GetParam();
    const double result = DiffAngle(tc.a, tc.b);
    EXPECT_NEAR(result, tc.expected, 1e-10)
        << "DiffAngle(" << tc.a << ", " << tc.b << ") = " << result
        << ", expected " << tc.expected;
}

INSTANTIATE_TEST_SUITE_P(
    AllCases,
    DiffAngleTest,
    ::testing::Values(
        DiffAngleTestParams{"ZeroZero",           0.0,          0.0,          0.0},
        DiffAngleTestParams{"SamePositive",        1.0,          1.0,          0.0},
        DiffAngleTestParams{"SameNegative",       -1.0,         -1.0,          0.0},
        DiffAngleTestParams{"SmallPositive",       0.5,          0.0,          0.5},
        DiffAngleTestParams{"SmallNegative",       0.0,          0.5,         -0.5},
        DiffAngleTestParams{"PiMinusEps",          M_PI - 0.1,   0.0,          M_PI - 0.1},
        DiffAngleTestParams{"NegPiPlusEps",      -(M_PI - 0.1),  0.0,        -(M_PI - 0.1)},
        DiffAngleTestParams{"ExactPi",             M_PI,         0.0,         -M_PI},
        DiffAngleTestParams{"ExactNegPi",         -M_PI,         0.0,         -M_PI},
        DiffAngleTestParams{"WrapPositive",        2.0 * M_PI,   0.0,          0.0},
        DiffAngleTestParams{"WrapNegative",       -2.0 * M_PI,   0.0,          0.0},
        DiffAngleTestParams{"AcrossWrapPos",       M_PI + 0.1,   0.0,        -(M_PI - 0.1)},
        DiffAngleTestParams{"AcrossWrapNeg",     -(M_PI + 0.1),  0.0,          M_PI - 0.1},
        DiffAngleTestParams{"BothPositiveLarge",   2.0 * M_PI,   M_PI,        -M_PI},
        DiffAngleTestParams{"BothNegativeLarge",  -2.0 * M_PI,  -M_PI,       -M_PI},
        DiffAngleTestParams{"OppositeExtremes",    2.0 * M_PI,  -2.0 * M_PI,  0.0},
        DiffAngleTestParams{"QuarterTurn",         M_PI / 2.0,   0.0,          M_PI / 2.0},
        DiffAngleTestParams{"NegQuarterTurn",      0.0,          M_PI / 2.0, -M_PI / 2.0},
        DiffAngleTestParams{"ThreeQuarters",       3.0 * M_PI / 2.0, 0.0,    -M_PI / 2.0},
        DiffAngleTestParams{"NegThreeQuarters",    0.0, 3.0 * M_PI / 2.0,     M_PI / 2.0},
        DiffAngleTestParams{"SymmetricSmall",      0.1,         -0.1,          0.2},
        DiffAngleTestParams{"SymmetricLarge",     -0.1,          0.1,         -0.2},
        DiffAngleTestParams{"MaxRange",            2.0 * M_PI,  -2.0 * M_PI,  0.0},
        DiffAngleTestParams{"MinRange",           -2.0 * M_PI,   2.0 * M_PI,  0.0}
    ),
    [](const ::testing::TestParamInfo<DiffAngleTestParams>& info) {
        return info.param.label;
    });

