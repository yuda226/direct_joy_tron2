#include <chrono>
#include <cmath>
#include <exception>
#include <cstdint>
#include <functional>
#include <memory>
#include <stdexcept>
#include <map>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "direct_joy/controller.hpp"
#include "direct_joy/mode_buttons.hpp"

namespace direct_joy
{

class DirectJoyNode : public rclcpp::Node
{
public:
  DirectJoyNode()
  : Node("direct_joy")
  {
    Config config;
    config.controller_mode = declare_parameter("controller_mode", config.controller_mode);
    config.enable_button = declare_parameter("enable_button", config.enable_button);
    config.max_speed_linear_x = declare_parameter("max_speed_linear_x", config.max_speed_linear_x);
    config.max_speed_linear_y = declare_parameter("max_speed_linear_y", config.max_speed_linear_y);
    config.max_speed_rotation = declare_parameter("max_speed_rotation", config.max_speed_rotation);
    config.turbo_scalar = declare_parameter("turbo_scalar", config.turbo_scalar);
    config.omni_drive = declare_parameter("omni_drive", config.omni_drive);
    config.deadzone = declare_parameter("deadzone", config.deadzone);
    config.joy_timeout = declare_parameter("joy_timeout", config.joy_timeout);
    const double rate = declare_parameter("publish_rate", 20.0);
    if (!std::isfinite(rate) || rate < 1.0 || rate > 1000.0) {
      throw std::invalid_argument("publish_rate must be between 1 and 1000 Hz");
    }
    controller_ = std::make_unique<Controller>(config);
    joy_timeout_ = config.joy_timeout;
    const bool modes_enabled = declare_parameter("mode_controls", false);
    service_timeout_ = declare_parameter("mode_service_timeout", 5.0);
    if (!std::isfinite(service_timeout_) || service_timeout_ <= 0.0) {
      throw std::invalid_argument("mode_service_timeout must be finite and positive");
    }
    if (modes_enabled) {
      mode_buttons_ = std::make_unique<ModeButtons>();
      const auto prefix = declare_parameter<std::string>(
        "mode_service_prefix", "/tron2_control_mode");
      for (const auto & name : {"stand", "walk", "sitdown", "recover", "emgy_stop"}) {
        triggers_[name] = create_client<Trigger>(prefix + "/" + name);
      }
      for (const auto & name : {"stair_mode", "base_height"}) {
        booleans_[name] = create_client<SetBool>(prefix + "/" + name);
      }
      RCLCPP_WARN(
        get_logger(), "Joystick mode controls enabled. Stair state assumes initially OFF; "
        "service acceptance does not confirm completed physical motion.");
    }
    publisher_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);
    subscription_ = create_subscription<sensor_msgs::msg::Joy>(
      "joy1/joy", rclcpp::SensorDataQoS().keep_last(1),
      [this](sensor_msgs::msg::Joy::ConstSharedPtr msg) {
        const auto error = controller_->update(*msg, Controller::Clock::now());
        if (!error.empty()) {
          if (mode_buttons_) {
            mode_buttons_->reset();
          }
          RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "%s", error.c_str());
        } else if (mode_buttons_) {
          const auto input = mode_buttons_->update(
            *msg, controller_->dMode(), Controller::Clock::now(), joy_timeout_);
          if (input.inhibit_drive || pending_ || emergency_latched_) {
            controller_->inhibit();
          }
          // Stop velocity before issuing any mode service request.
          publisher_->publish(controller_->command());
          requestMode(input.action);
        }
        if (error.empty() && !controller_->ready()) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 5000,
            "Center both sticks and release the enable button before driving");
        }
        publisher_->publish(controller_->command());
      });
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / rate)),
      [this]() {
        checkRequestTimeout();
        if (controller_->expire(Controller::Clock::now())) {
          RCLCPP_WARN(get_logger(), "Joystick timeout: stopped; release enable and center sticks");
        }
        publisher_->publish(controller_->command());
      });
    RCLCPP_INFO(
      get_logger(), "Hold button %d to drive: %s -> %s. No I2R safety filtering.",
      config.enable_button, subscription_->get_topic_name(), publisher_->get_topic_name());
  }

private:
  using Trigger = std_srvs::srv::Trigger;
  using SetBool = std_srvs::srv::SetBool;

  template<typename Service>
  void sendRequest(
    const typename rclcpp::Client<Service>::SharedPtr & client,
    const std::shared_ptr<typename Service::Request> & request,
    bool stairs = false, bool recovery = false)
  {
    const std::string name = client->get_service_name();
    if (!client->service_is_ready()) {
      RCLCPP_ERROR(get_logger(), "%s unavailable; release combination and retry", name.c_str());
      return;
    }
    const auto generation = ++generation_;
    pending_ = true;
    deadline_ = Controller::Clock::now() +
      std::chrono::duration_cast<Controller::Clock::duration>(
      std::chrono::duration<double>(service_timeout_));
    auto future = client->async_send_request(
      request, [this, generation, name, stairs, recovery](
        typename rclcpp::Client<Service>::SharedFuture reply) {
        if (generation != generation_) {
          return;
        }
        pending_ = false;
        remove_pending_ = {};
        const auto response = reply.get();
        if (response->success) {
          if (stairs) {
            stairs_enabled_ = !stairs_enabled_;
          }
          if (recovery) {
            emergency_latched_ = false;
          }
          RCLCPP_INFO(
            get_logger(), "%s accepted (not physical completion): %s",
            name.c_str(), response->message.c_str());
        } else {
          RCLCPP_ERROR(
            get_logger(), "%s failed: %s; inspect robot before retrying",
            name.c_str(), response->message.c_str());
        }
      });
    remove_pending_ = [client, id = future.request_id]() {client->remove_pending_request(id);};
    RCLCPP_INFO(get_logger(), "Requested %s", name.c_str());
  }

  void checkRequestTimeout()
  {
    if (pending_ && Controller::Clock::now() >= deadline_) {
      ++generation_;
      remove_pending_();
      remove_pending_ = {};
      pending_ = false;
      RCLCPP_ERROR(
        get_logger(), "Mode service timed out; robot outcome unknown. Inspect robot before retrying.");
    }
  }

  void requestMode(ModeAction action)
  {
    if (action == ModeAction::none) {
      return;
    }
    if (action == ModeAction::emergency) {
      emergency_latched_ = true;
    }
    if (pending_) {
      if (action != ModeAction::emergency) {
        RCLCPP_WARN(get_logger(), "Mode request pending; release combination and retry when complete");
        return;
      }
      ++generation_;
      remove_pending_();
      remove_pending_ = {};
      pending_ = false;
      RCLCPP_WARN(get_logger(), "Interrupting local request wait for emergency stop; outcome unknown");
    }
    switch (action) {
      case ModeAction::stand:
        sendRequest<Trigger>(triggers_.at("stand"), std::make_shared<Trigger::Request>());
        break;
      case ModeAction::walk:
        sendRequest<Trigger>(triggers_.at("walk"), std::make_shared<Trigger::Request>());
        break;
      case ModeAction::sitdown:
        sendRequest<Trigger>(triggers_.at("sitdown"), std::make_shared<Trigger::Request>());
        break;
      case ModeAction::recover:
        sendRequest<Trigger>(
          triggers_.at("recover"), std::make_shared<Trigger::Request>(), false, true);
        break;
      case ModeAction::emergency:
        sendRequest<Trigger>(triggers_.at("emgy_stop"), std::make_shared<Trigger::Request>());
        break;
      case ModeAction::stairs:
      case ModeAction::raise:
      case ModeAction::lower: {
        auto request = std::make_shared<SetBool::Request>();
        request->data = action == ModeAction::stairs ? !stairs_enabled_ : action == ModeAction::raise;
        sendRequest<SetBool>(
          booleans_.at(action == ModeAction::stairs ? "stair_mode" : "base_height"),
          request, action == ModeAction::stairs);
        break;
      }
      case ModeAction::none:
        break;
    }
  }

  std::unique_ptr<ModeButtons> mode_buttons_;
  std::map<std::string, rclcpp::Client<Trigger>::SharedPtr> triggers_;
  std::map<std::string, rclcpp::Client<SetBool>::SharedPtr> booleans_;
  double joy_timeout_{0.5};
  double service_timeout_{5.0};
  bool pending_{false};
  bool stairs_enabled_{false};
  bool emergency_latched_{false};
  uint64_t generation_{0};
  Controller::TimePoint deadline_{};
  std::function<void()> remove_pending_;
  std::unique_ptr<Controller> controller_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace direct_joy

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<direct_joy::DirectJoyNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("direct_joy"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
