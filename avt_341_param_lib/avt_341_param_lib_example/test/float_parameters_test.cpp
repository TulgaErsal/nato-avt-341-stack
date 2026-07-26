#include <type_traits>
#include <utility>
#include <vector>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include <avt_341_param_lib_example/float_demo_params_service.hpp>

static_assert(
    std::is_same_v<
        decltype(std::declval<float_demo::Params>().default_scalar),
        float>);
static_assert(
    std::is_same_v<
        decltype(std::declval<float_demo::Params>().default_array),
        std::vector<float>>);

class FloatParametersTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestSuite() {
    rclcpp::shutdown();
  }
};

TEST_F(FloatParametersTest, InitializesAndUpdatesUsingRosDoubleParameters) {
  const double required_scalar = 2.75;
  const std::vector<double> required_array{3.125, -4.5};

  auto options = rclcpp::NodeOptions().parameter_overrides({
      rclcpp::Parameter("required_scalar", required_scalar),
      rclcpp::Parameter("required_array", required_array),
  });
  auto node = std::make_shared<rclcpp::Node>(
      "float_parameter_test", options);
  float_demo::ParamsListener listener(node);

  auto params = listener.get_params();
  EXPECT_FLOAT_EQ(params.default_scalar, 1.25F);
  EXPECT_EQ(params.default_array, (std::vector<float>{1.5F, -2.25F}));
  EXPECT_FLOAT_EQ(
      params.required_scalar, static_cast<float>(required_scalar));
  EXPECT_EQ(
      params.required_array,
      (std::vector<float>{
          static_cast<float>(required_array[0]),
          static_cast<float>(required_array[1])}));
  EXPECT_FALSE(listener.is_old(params));
  auto stale_params = params;
  const auto initial_stamp = params.__stamp;

  const double updated_scalar = 6.125;
  const std::vector<double> updated_array{7.75, -8.875};
  const auto result = node->set_parameters_atomically({
      rclcpp::Parameter("required_scalar", updated_scalar),
      rclcpp::Parameter("required_array", updated_array),
  });
  ASSERT_TRUE(result.successful) << result.reason;

  EXPECT_TRUE(listener.is_old(stale_params));
  EXPECT_TRUE(listener.try_update_params(stale_params));
  EXPECT_FALSE(listener.try_update_params(stale_params));
  EXPECT_NE(initial_stamp, stale_params.__stamp);

  params = listener.get_params();
  EXPECT_FLOAT_EQ(
      params.required_scalar, static_cast<float>(updated_scalar));
  EXPECT_EQ(
      params.required_array,
      (std::vector<float>{
          static_cast<float>(updated_array[0]),
          static_cast<float>(updated_array[1])}));

  const auto invalid_result = node->set_parameter(
      rclcpp::Parameter("required_scalar", 11.0));
  EXPECT_FALSE(invalid_result.successful);
  EXPECT_EQ(listener.get_params().__stamp, params.__stamp);
  EXPECT_FLOAT_EQ(
      listener.get_params().required_scalar,
      static_cast<float>(updated_scalar));
}
