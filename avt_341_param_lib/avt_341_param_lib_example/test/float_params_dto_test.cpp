#include <cstdint>
#include <type_traits>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include <avt_341_param_lib_example/float_demo_params_dto.hpp>

static_assert(
    std::is_same_v<
        decltype(std::declval<float_demo::Params>().default_scalar),
        float>);
static_assert(
    std::is_same_v<
        decltype(std::declval<float_demo::Params>().default_array),
        std::vector<float>>);
static_assert(std::is_standard_layout_v<float_demo::Params>);
static_assert(std::is_copy_constructible_v<float_demo::Params>);
static_assert(std::is_copy_assignable_v<float_demo::Params>);
static_assert(std::is_standard_layout_v<float_demo::ParamsStamp>);
static_assert(std::is_trivially_copyable_v<float_demo::ParamsStamp>);

TEST(FloatParamsDtoTest, StampIsRosFreeValueType) {
  const float_demo::ParamsStamp initial{};
  const float_demo::ParamsStamp same{};
  const float_demo::ParamsStamp later{12, 345U};

  EXPECT_EQ(initial.sec, std::int32_t{0});
  EXPECT_EQ(initial.nanosec, std::uint32_t{0});
  EXPECT_EQ(initial, same);
  EXPECT_NE(initial, later);
}
