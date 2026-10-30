#include <limits>

#include <gtest/gtest.h>

#include "direct_joy/controller.hpp"

using direct_joy::Config;
using direct_joy::Controller;

namespace
{
sensor_msgs::msg::Joy joystick(bool d_mode = false)
{
  sensor_msgs::msg::Joy joy;
  joy.axes.resize(d_mode ? 6 : 8, 0.0F);
  joy.buttons.resize(d_mode ? 12 : 11, 0);
  if (!d_mode) {
    joy.axes[2] = 1.0F;
    joy.axes[5] = 1.0F;
  }
  return joy;
}

const Controller::TimePoint start{};
}  // namespace

TEST(Controller, RequiresReleasedEnableAndCenteredSticks)
{
  Controller controller(Config{});
  auto joy = joystick();
  joy.buttons[5] = 1;
  joy.axes[1] = 1.0F;
  EXPECT_TRUE(controller.update(joy, start).empty());
  EXPECT_DOUBLE_EQ(controller.command().linear.x, 0.0);
  joy.buttons[5] = 0;
  controller.update(joy, start);
  EXPECT_FALSE(controller.ready());
  joy.axes[1] = 0.0F;
  controller.update(joy, start);
  EXPECT_TRUE(controller.ready());
  joy.buttons[5] = 1;
  joy.axes[1] = 1.0F;
  controller.update(joy, start);
  EXPECT_NEAR(controller.command().linear.x, 1.0 / 1.2, 1e-9);
  joy.buttons[5] = 0;
  controller.update(joy, start);
  EXPECT_DOUBLE_EQ(controller.command().linear.x, 0.0);
}

TEST(Controller, PreservesXModeMappingScalingAndTurbo)
{
  Controller controller(Config{});
  auto joy = joystick();
  controller.update(joy, start);
  joy.buttons[5] = 1;
  joy.axes[0] = 1.0F;
  joy.axes[1] = -0.6F;
  joy.axes[3] = 0.3F;
  controller.update(joy, start);
  EXPECT_NEAR(controller.command().linear.x, -0.5, 1e-6);
  EXPECT_DOUBLE_EQ(controller.command().linear.y, 0.0);
  EXPECT_NEAR(controller.command().angular.z, 0.25, 1e-6);
  joy.axes[2] = -1.0F;
  controller.update(joy, start);
  EXPECT_NEAR(controller.command().linear.x, -0.6, 1e-6);
  EXPECT_NEAR(controller.command().angular.z, 0.3, 1e-6);
}

TEST(Controller, SupportsDModeAndOmniDrive)
{
  Config config;
  config.omni_drive = true;
  config.max_speed_linear_y = 0.5;
  Controller controller(config);
  auto joy = joystick(true);
  controller.update(joy, start);
  joy.buttons[5] = 1;
  joy.buttons[6] = 1;
  joy.axes[0] = -1.0F;
  joy.axes[1] = 1.0F;
  joy.axes[2] = -1.0F;
  controller.update(joy, start);
  EXPECT_DOUBLE_EQ(controller.command().linear.x, 1.0);
  EXPECT_DOUBLE_EQ(controller.command().linear.y, -0.5);
  EXPECT_DOUBLE_EQ(controller.command().angular.z, -1.0);
}

TEST(Controller, StopsAtTimeoutAndRequiresRearming)
{
  Controller controller(Config{});
  auto joy = joystick();
  controller.update(joy, start);
  joy.buttons[5] = 1;
  joy.axes[1] = 1.0F;
  controller.update(joy, start);
  EXPECT_FALSE(controller.expire(start + std::chrono::milliseconds(499)));
  EXPECT_GT(controller.command().linear.x, 0.0);
  EXPECT_TRUE(controller.expire(start + std::chrono::milliseconds(500)));
  EXPECT_DOUBLE_EQ(controller.command().linear.x, 0.0);
  controller.update(joy, start + std::chrono::milliseconds(501));
  EXPECT_DOUBLE_EQ(controller.command().linear.x, 0.0);
  joy.buttons[5] = 0;
  joy.axes[1] = 0.0F;
  controller.update(joy, start + std::chrono::milliseconds(502));
  EXPECT_TRUE(controller.ready());
}

TEST(Controller, ChecksTimeoutEvenBeforeTimerRuns)
{
  Controller controller(Config{});
  auto joy = joystick();
  controller.update(joy, start);
  joy.buttons[5] = 1;
  joy.axes[1] = 1.0F;
  controller.update(joy, start + std::chrono::seconds(1));
  EXPECT_DOUBLE_EQ(controller.command().linear.x, 0.0);
}

TEST(Controller, RejectsMalformedAndNonfiniteMessages)
{
  Controller controller(Config{});
  EXPECT_FALSE(controller.update(sensor_msgs::msg::Joy{}, start).empty());
  auto joy = joystick();
  controller.update(joy, start);
  joy.buttons[5] = 1;
  joy.axes[1] = 1.0F;
  controller.update(joy, start);
  joy.axes[1] = std::numeric_limits<float>::quiet_NaN();
  EXPECT_FALSE(controller.update(joy, start).empty());
  EXPECT_DOUBLE_EQ(controller.command().linear.x, 0.0);
  EXPECT_FALSE(controller.ready());
  joy.axes[1] = std::numeric_limits<float>::infinity();
  EXPECT_FALSE(controller.update(joy, start).empty());
  joy.axes[1] = 1.1F;
  EXPECT_FALSE(controller.update(joy, start).empty());
  joy.axes[1] = 0.0F;
  joy.buttons[0] = 2;
  EXPECT_FALSE(controller.update(joy, start).empty());
}

TEST(Controller, ModeChangesRequireRearming)
{
  Controller controller(Config{});
  controller.update(joystick(), start);
  auto joy = joystick(true);
  joy.buttons[5] = 1;
  joy.axes[1] = 1.0F;
  controller.update(joy, start);
  EXPECT_FALSE(controller.ready());
  EXPECT_DOUBLE_EQ(controller.command().linear.x, 0.0);
}

TEST(Controller, AppliesDeadzoneAndLeavesUnusedComponentsZero)
{
  Controller controller(Config{});
  auto joy = joystick();
  controller.update(joy, start);
  joy.buttons[5] = 1;
  joy.axes[1] = 0.04F;
  joy.axes[3] = -0.04F;
  controller.update(joy, start);
  EXPECT_DOUBLE_EQ(controller.command().linear.x, 0.0);
  EXPECT_DOUBLE_EQ(controller.command().linear.z, 0.0);
  EXPECT_DOUBLE_EQ(controller.command().angular.x, 0.0);
  EXPECT_DOUBLE_EQ(controller.command().angular.y, 0.0);
  EXPECT_DOUBLE_EQ(controller.command().angular.z, 0.0);
}

TEST(Controller, RejectsInvalidConfiguration)
{
  Config config;
  config.controller_mode = "unknown";
  EXPECT_THROW(Controller{config}, std::invalid_argument);
  config = Config{};
  config.joy_timeout = 0.0;
  EXPECT_THROW(Controller{config}, std::invalid_argument);
  config = Config{};
  config.turbo_scalar = 0.5;
  EXPECT_THROW(Controller{config}, std::invalid_argument);
  config = Config{};
  config.max_speed_linear_x = std::numeric_limits<double>::infinity();
  EXPECT_THROW(Controller{config}, std::invalid_argument);
  config = Config{};
  config.enable_button = -1;
  EXPECT_THROW(Controller{config}, std::invalid_argument);
}

TEST(Controller, SupportsExplicitModeAndCustomEnableButton)
{
  Config config;
  config.controller_mode = "x";
  config.enable_button = 12;
  Controller controller(config);
  auto joy = joystick();
  EXPECT_FALSE(controller.update(joy, start).empty());
  joy.buttons.resize(13, 0);
  EXPECT_TRUE(controller.update(joy, start).empty());
  joy.buttons[12] = 1;
  joy.axes[1] = 1.0F;
  EXPECT_TRUE(controller.update(joy, start).empty());
  EXPECT_GT(controller.command().linear.x, 0.0);
}
