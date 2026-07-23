#include <gtest/gtest.h>

#include "avt_341/perception/perception_settings.hpp"

namespace avt_341::perception {
namespace {

TEST(PerceptionSettingsTest, InheritsGeneratedFieldsAndComputesGeometry) {
  GeneratedPerceptionParams params;
  params.costmap.size_info.width = 10.0F;
  params.costmap.size_info.height = 8.0F;
  params.costmap.size_info.res = 0.5F;
  params.costmap.size_info.llx = -2.0F;
  params.costmap.size_info.lly = -1.0F;
  params.costmap.dilation.enabled = true;
  params.costmap.dilation.x = 1.0F;
  params.costmap.dilation.y = 1.5F;
  params.costmap.terrain_rms.time_window = 0.25F;
  params.runtime.rate = 20.0;

  const PerceptionSettings settings(params);

  EXPECT_FLOAT_EQ(settings.costmap.size_info.res, 0.5F);
  EXPECT_EQ(settings.nx(), 20);
  EXPECT_EQ(settings.ny(), 16);
  EXPECT_EQ(settings.dilation_x_cells(), 2);
  EXPECT_EQ(settings.dilation_y_cells(), 3);
  EXPECT_EQ(settings.rms_window_samples(), 5);
  EXPECT_FLOAT_EQ(settings.to_x_world(0), -1.75F);
  EXPECT_FLOAT_EQ(settings.to_y_world(0), -0.75F);
}

TEST(PerceptionSettingsTest, BaseSnapshotUpdateCannotStaleComputedValues) {
  PerceptionSettings settings;
  settings.costmap.size_info.width = 10.0F;
  settings.costmap.size_info.res = 1.0F;
  EXPECT_EQ(settings.nx(), 10);

  GeneratedPerceptionParams updated;
  updated.costmap.size_info.width = 12.0F;
  updated.costmap.size_info.res = 0.5F;

  static_cast<GeneratedPerceptionParams&>(settings) = updated;

  EXPECT_EQ(settings.nx(), 24);
}

TEST(PerceptionSettingsTest, ThresholdUpdatesUseGeneratedStorage) {
  PerceptionSettings settings;
  settings.costmap.thresholds.thresh = 0.5F;
  settings.costmap.thresholds.thresh_max = 2.5F;

  settings.update_thresholds(1.0F, 4.0F);

  EXPECT_FLOAT_EQ(settings.costmap.thresholds.thresh, 1.0F);
  EXPECT_FLOAT_EQ(settings.costmap.thresholds.thresh_max, 4.0F);
  EXPECT_FLOAT_EQ(settings.grid_slope_multiplier(), 100.0F / 3.0F);
}

}  // namespace
}  // namespace avt_341::perception
