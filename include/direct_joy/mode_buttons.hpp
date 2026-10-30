#ifndef DIRECT_JOY__MODE_BUTTONS_HPP_
#define DIRECT_JOY__MODE_BUTTONS_HPP_

#include <optional>

#include "direct_joy/controller.hpp"

namespace direct_joy
{

enum class ModeAction {none, stand, walk, sitdown, recover, emergency, stairs, raise, lower};

struct ModeInput
{
  ModeAction action{ModeAction::none};
  bool inhibit_drive{false};
};

// Input has already passed Controller's layout and value validation.
class ModeButtons
{
public:
  void reset()
  {
    armed_ = false;
    fired_ = false;
    emergency_held_ = false;
    stair_start_.reset();
    last_input_.reset();
  }

  ModeInput update(
    const sensor_msgs::msg::Joy & joy, bool d_mode, Controller::TimePoint now,
    double timeout)
  {
    if ((last_input_ &&
      std::chrono::duration<double>(now - *last_input_).count() >= timeout) ||
      (last_d_mode_ && *last_d_mode_ != d_mode))
    {
      reset();
    }
    last_input_ = now;
    last_d_mode_ = d_mode;
    const bool a = joy.buttons[d_mode ? 1 : 0] == 1;
    const bool b = joy.buttons[d_mode ? 2 : 1] == 1;
    const bool x = joy.buttons[d_mode ? 0 : 2] == 1;
    const bool y = joy.buttons[3] == 1;
    const bool lb = joy.buttons[4] == 1;
    const bool rb = joy.buttons[5] == 1;
    const bool emergency = joy.buttons[d_mode ? 10 : 9] == 1 &&
      joy.buttons[d_mode ? 11 : 10] == 1;
    const bool active = lb || x || (rb && (a || b || y)) || (a && y) || emergency;
    if (!active) {
      armed_ = true;
      fired_ = false;
      emergency_held_ = false;
      stair_start_.reset();
      return {};
    }
    ModeInput result{ModeAction::none, true};
    // Emergency requests can interrupt an already-consumed non-emergency chord.
    if (emergency && !emergency_held_) {
      emergency_held_ = true;
      fired_ = true;
      stair_start_.reset();
      result.action = ModeAction::emergency;
      return result;
    }
    if (!armed_ || fired_ || emergency) {
      return result;
    }
    if (a && y && !lb && !x && !b) {
      if (!stair_start_) {
        stair_start_ = now;
      }
      if (now - *stair_start_ >= std::chrono::seconds(2)) {
        result.action = ModeAction::stairs;
      }
    } else {
      stair_start_.reset();
      if (lb && rb && y && !x && !a && !b) {
        result.action = ModeAction::recover;
      } else if (rb && !lb && !x) {
        if (y && !a && !b) {
          result.action = ModeAction::stand;
        } else if (b && !a && !y) {
          result.action = ModeAction::walk;
        } else if (a && !b && !y) {
          result.action = ModeAction::sitdown;
        }
      } else if (x && !lb && !a && !b && !y && !rb) {
        const auto vertical = joy.axes[d_mode ? 5 : 7];
        if (vertical > 0.5F) {
          result.action = ModeAction::raise;
        } else if (vertical < -0.5F) {
          result.action = ModeAction::lower;
        }
      }
    }
    if (result.action != ModeAction::none) {
      fired_ = true;
    }
    return result;
  }

private:
  bool armed_{false};
  bool fired_{false};
  bool emergency_held_{false};
  std::optional<bool> last_d_mode_;
  std::optional<Controller::TimePoint> last_input_;
  std::optional<Controller::TimePoint> stair_start_;
};

}  // namespace direct_joy

#endif  // DIRECT_JOY__MODE_BUTTONS_HPP_
