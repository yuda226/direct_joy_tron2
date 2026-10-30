#include <gtest/gtest.h>

#include "direct_joy/mode_buttons.hpp"

using direct_joy::Controller;
using direct_joy::ModeAction;
using direct_joy::ModeButtons;

namespace
{
sensor_msgs::msg::Joy joystick(bool d = false)
{
  sensor_msgs::msg::Joy joy;
  joy.axes.resize(d ? 6 : 8, 0.0F);
  joy.buttons.resize(d ? 12 : 11, 0);
  return joy;
}

const Controller::TimePoint start{};
}  // namespace

TEST(ModeButtons, MapsBothLayoutsAndConsumesEachChordUntilRelease)
{
  for (const bool d : {false, true}) {
    ModeButtons buttons;
    auto joy = joystick(d);
    auto update = [&]() {return buttons.update(joy, d, start, 0.5);};
    EXPECT_FALSE(update().inhibit_drive);
    joy.buttons[4] = 1;
    joy.buttons[3] = 1;
    EXPECT_EQ(update().action, ModeAction::stand);
    EXPECT_EQ(update().action, ModeAction::none);
    joy.buttons[d ? 2 : 1] = 1;
    EXPECT_EQ(update().action, ModeAction::none);
    joy = joystick(d);
    update();
    joy.buttons[4] = 1;
    joy.buttons[d ? 2 : 1] = 1;
    EXPECT_EQ(update().action, ModeAction::walk);
    joy = joystick(d);
    update();
    joy.buttons[4] = 1;
    joy.buttons[d ? 1 : 0] = 1;
    EXPECT_EQ(update().action, ModeAction::sitdown);
    joy = joystick(d);
    update();
    joy.buttons[4] = joy.buttons[5] = joy.buttons[3] = 1;
    EXPECT_EQ(update().action, ModeAction::recover);
    joy.buttons[5] = 0;
    EXPECT_EQ(update().action, ModeAction::none);
    joy = joystick(d);
    update();
    joy.buttons[4] = 1;
    joy.buttons[d ? 0 : 2] = 1;
    EXPECT_EQ(update().action, ModeAction::none);  // Idle is intentionally unassigned.
    EXPECT_TRUE(update().inhibit_drive);
  }
}

TEST(ModeButtons, StairHoldRequiresTwoContinuousSecondsAndFreshInput)
{
  ModeButtons buttons;
  auto joy = joystick();
  buttons.update(joy, false, start, 0.5);
  joy.buttons[0] = joy.buttons[3] = 1;
  for (int ms = 0; ms < 2000; ms += 100) {
    const auto input = buttons.update(joy, false, start + std::chrono::milliseconds(ms), 0.5);
    EXPECT_EQ(input.action, ModeAction::none);
    EXPECT_TRUE(input.inhibit_drive);
  }
  EXPECT_EQ(
    buttons.update(joy, false, start + std::chrono::milliseconds(1999), 0.5).action,
    ModeAction::none);
  EXPECT_EQ(
    buttons.update(joy, false, start + std::chrono::seconds(2), 0.5).action,
    ModeAction::stairs);
  EXPECT_EQ(
    buttons.update(joy, false, start + std::chrono::milliseconds(2100), 0.5).action,
    ModeAction::none);
  buttons.reset();
  buttons.update(joystick(), false, start, 0.5);
  buttons.update(joy, false, start, 0.5);
  EXPECT_EQ(
    buttons.update(joy, false, start + std::chrono::seconds(2), 0.5).action,
    ModeAction::none);
}

TEST(ModeButtons, HeightIsOneShotAndDpadAloneDoesNotBlockDriving)
{
  for (const bool d : {false, true}) {
    ModeButtons buttons;
    auto joy = joystick(d);
    const auto vertical = d ? 5 : 7;
    joy.axes[vertical] = 1.0F;
    EXPECT_FALSE(buttons.update(joy, d, start, 0.5).inhibit_drive);
    joy.buttons[d ? 0 : 2] = 1;
    EXPECT_EQ(buttons.update(joy, d, start, 0.5).action, ModeAction::raise);
    EXPECT_EQ(buttons.update(joy, d, start, 0.5).action, ModeAction::none);
    joy.axes[vertical] = -1.0F;
    EXPECT_EQ(buttons.update(joy, d, start, 0.5).action, ModeAction::none);
    buttons.update(joystick(d), d, start, 0.5);
    EXPECT_EQ(buttons.update(joy, d, start, 0.5).action, ModeAction::lower);
  }
}

TEST(ModeButtons, EmergencyPreemptsOtherChordsAndDoesNotToggleRecovery)
{
  for (const bool d : {false, true}) {
    ModeButtons buttons;
    auto joy = joystick(d);
    buttons.update(joy, d, start, 0.5);
    joy.buttons[4] = joy.buttons[3] = 1;
    EXPECT_EQ(buttons.update(joy, d, start, 0.5).action, ModeAction::stand);
    joy.buttons[d ? 10 : 9] = joy.buttons[d ? 11 : 10] = 1;
    EXPECT_EQ(buttons.update(joy, d, start, 0.5).action, ModeAction::emergency);
    EXPECT_EQ(buttons.update(joy, d, start, 0.5).action, ModeAction::none);
    buttons.update(joystick(d), d, start, 0.5);
    EXPECT_EQ(buttons.update(joy, d, start, 0.5).action, ModeAction::emergency);
  }
}

TEST(ModeButtons, StartupTimeoutResetAndLayoutChangeRequireNeutral)
{
  ModeButtons buttons;
  auto joy = joystick();
  joy.buttons[4] = joy.buttons[3] = 1;
  EXPECT_EQ(buttons.update(joy, false, start, 0.5).action, ModeAction::none);
  buttons.update(joystick(), false, start, 0.5);
  EXPECT_EQ(
    buttons.update(joy, false, start + std::chrono::milliseconds(500), 0.5).action,
    ModeAction::none);
  buttons.update(joystick(), false, start + std::chrono::milliseconds(501), 0.5);
  buttons.reset();
  EXPECT_EQ(
    buttons.update(joy, false, start + std::chrono::milliseconds(502), 0.5).action,
    ModeAction::none);
  buttons.update(joystick(), false, start, 0.5);
  auto d = joystick(true);
  d.buttons[4] = d.buttons[3] = 1;
  EXPECT_EQ(buttons.update(d, true, start, 0.5).action, ModeAction::none);
}

TEST(ModeButtons, ReleasedOrConflictingStairChordCannotTrigger)
{
  ModeButtons buttons;
  auto joy = joystick();
  buttons.update(joy, false, start, 0.5);
  joy.buttons[0] = joy.buttons[3] = 1;
  buttons.update(joy, false, start, 0.5);
  joy.buttons[0] = 0;
  buttons.update(joy, false, start + std::chrono::milliseconds(100), 0.5);
  joy.buttons[0] = 1;
  for (int ms = 200; ms < 2200; ms += 100) {
    EXPECT_EQ(
      buttons.update(joy, false, start + std::chrono::milliseconds(ms), 0.5).action,
      ModeAction::none);
  }
  EXPECT_EQ(
    buttons.update(joy, false, start + std::chrono::milliseconds(2200), 0.5).action,
    ModeAction::stairs);
  buttons.reset();
  buttons.update(joystick(), false, start, 0.5);
  joy.buttons[4] = 1;
  for (int ms = 0; ms <= 2100; ms += 100) {
    EXPECT_EQ(
      buttons.update(joy, false, start + std::chrono::milliseconds(ms), 0.5).action,
      ModeAction::none);
  }
}

TEST(Controller, ModeInhibitionRequiresReleasedEnableAndCenteredSticks)
{
  Controller controller(direct_joy::Config{});
  auto joy = joystick();
  controller.update(joy, start);
  joy.buttons[5] = 1;
  joy.axes[1] = 1.0F;
  controller.update(joy, start);
  EXPECT_GT(controller.command().linear.x, 0.0);
  controller.inhibit();
  controller.update(joy, start);
  EXPECT_FALSE(controller.ready());
  EXPECT_DOUBLE_EQ(controller.command().linear.x, 0.0);
  controller.update(joystick(), start);
  EXPECT_TRUE(controller.ready());
}
