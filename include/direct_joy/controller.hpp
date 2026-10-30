#ifndef DIRECT_JOY__CONTROLLER_HPP_
#define DIRECT_JOY__CONTROLLER_HPP_

#include <chrono>
#include <cmath>
#include <optional>
#include <stdexcept>
#include <string>

#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joy.hpp>

namespace direct_joy
{

struct Config
{
  std::string controller_mode{"auto"};
  int enable_button{5};
  double max_speed_linear_x{1.0};
  double max_speed_linear_y{1.0};
  double max_speed_rotation{1.0};
  double turbo_scalar{1.2};
  bool omni_drive{false};
  double deadzone{0.05};
  double joy_timeout{0.5};
};

// Retains the original controller's Logitech X/D mappings and speed scaling.
class Controller
{
public:
  using Clock = std::chrono::steady_clock;
  using TimePoint = Clock::time_point;

  explicit Controller(const Config & config)
  : config_(config)
  {
    if (config_.controller_mode != "auto" && config_.controller_mode != "x" &&
      config_.controller_mode != "d")
    {
      throw std::invalid_argument("controller_mode must be auto, x, or d");
    }
    if (config_.enable_button < 0 ||
      !positive(config_.max_speed_linear_x) || !positive(config_.max_speed_linear_y) ||
      !positive(config_.max_speed_rotation) || !positive(config_.joy_timeout) ||
      !std::isfinite(config_.turbo_scalar) || config_.turbo_scalar < 1.0 ||
      !std::isfinite(config_.deadzone) || config_.deadzone < 0.0 || config_.deadzone >= 1.0)
    {
      throw std::invalid_argument("Invalid speed, enable_button, turbo, deadzone, or timeout");
    }
  }

  // A nonempty result describes rejected input, which always clears motion.
  std::string update(const sensor_msgs::msg::Joy & joy, TimePoint now)
  {
    expire(now);
    bool d_mode = config_.controller_mode == "d";
    if (config_.controller_mode == "auto") {
      if (joy.axes.size() == 6 && joy.buttons.size() == 12) {
        d_mode = true;
      } else if (joy.axes.size() == 8 && joy.buttons.size() == 11) {
        d_mode = false;
      } else {
        stop();
        return "Unsupported controller layout; expected X (8 axes/11 buttons) or D (6/12)";
      }
    }
    const std::size_t required_axes = d_mode ? 6 : 8;
    const std::size_t required_buttons = d_mode ? 12 : 11;
    if (joy.axes.size() < required_axes || joy.buttons.size() < required_buttons ||
      static_cast<std::size_t>(config_.enable_button) >= joy.buttons.size())
    {
      stop();
      return "Joy message is too short for controller_mode or enable_button";
    }
    for (const auto axis : joy.axes) {
      if (!std::isfinite(axis) || axis < -1.0F || axis > 1.0F) {
        stop();
        return "Joy axes must be finite and within [-1, 1]";
      }
    }
    for (const auto button : joy.buttons) {
      if (button != 0 && button != 1) {
        stop();
        return "Joy buttons must be 0 or 1";
      }
    }
    if (last_d_mode_ && *last_d_mode_ != d_mode) {
      stop();
    }
    last_d_mode_ = d_mode;
    last_input_ = now;
    command_ = geometry_msgs::msg::Twist{};

    const std::size_t yaw_axis = d_mode ? 2 : 3;
    const std::size_t right_y_axis = d_mode ? 3 : 4;
    const bool enabled = joy.buttons[config_.enable_button] == 1;
    if (!enabled) {
      ready_ = std::abs(joy.axes[0]) <= config_.deadzone &&
        std::abs(joy.axes[1]) <= config_.deadzone &&
        std::abs(joy.axes[yaw_axis]) <= config_.deadzone &&
        std::abs(joy.axes[right_y_axis]) <= config_.deadzone;
      return {};
    }
    if (!ready_) {
      return {};
    }
    const bool turbo = d_mode ? joy.buttons[6] == 1 : joy.axes[2] < -0.5F;
    const double scale = turbo ? 1.0 : 1.0 / config_.turbo_scalar;
    command_.linear.x = axisValue(joy.axes[1]) * config_.max_speed_linear_x * scale;
    command_.linear.y = config_.omni_drive ?
      axisValue(joy.axes[0]) * config_.max_speed_linear_y * scale : 0.0;
    command_.angular.z = axisValue(joy.axes[yaw_axis]) * config_.max_speed_rotation * scale;
    return {};
  }

  bool expire(TimePoint now)
  {
    if (last_input_ &&
      std::chrono::duration<double>(now - *last_input_).count() >= config_.joy_timeout)
    {
      stop();
      return true;
    }
    return false;
  }

  const geometry_msgs::msg::Twist & command() const { return command_; }
  bool ready() const { return ready_; }
  bool dMode() const { return last_d_mode_.value_or(false); }
  void inhibit() { stop(); }

private:
  static bool positive(double value) { return std::isfinite(value) && value > 0.0; }
  double axisValue(double value) const
  {
    return std::abs(value) <= config_.deadzone ? 0.0 : value;
  }
  void stop()
  {
    ready_ = false;
    last_input_.reset();
    command_ = geometry_msgs::msg::Twist{};
  }

  Config config_;
  bool ready_{false};
  std::optional<bool> last_d_mode_;
  std::optional<TimePoint> last_input_;
  geometry_msgs::msg::Twist command_;
};

}  // namespace direct_joy

#endif  // DIRECT_JOY__CONTROLLER_HPP_
