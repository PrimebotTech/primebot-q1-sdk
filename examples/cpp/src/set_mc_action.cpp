/*
 MC Action and Motion Control Example Script
 
 Description:
   This script demonstrates how to control robot actions and motions using the SetMcAction and SetMcMotion services.
   Supports both interactive action mode and automatic motion execution with state machine transitions.
 
 Prerequisites:
   - MC (Motion Control) service must be running
   - Robot must be in a safe environment for motion testing
   - McActionSwitcher plans supported state transitions
 
 Usage:
   ros2 run aimdk_examples_cpp set_mc_action --ros-args -p type:=<type> -p motion:=<motion_name>
 
 Example:
   # Interactive action mode
   ros2 run aimdk_examples_cpp set_mc_action --ros-args -p type:=action
   
   # Execute specific motion with auto-transition
   ros2 run aimdk_examples_cpp set_mc_action --ros-args -p type:=motion -p motion:=INTRO_POSE6
 
 Parameters:
   - type: Control type, either 'action' or 'motion' (required)
   - action_desc: Action description string (used in action mode)
   - motion: Motion name to execute (required when type=motion)
 */
#include "aimdk_msgs/msg/common_request.hpp"
#include "aimdk_msgs/msg/common_state.hpp"
#include "aimdk_msgs/msg/mc_action_status.hpp"
#include "aimdk_msgs/srv/get_mc_action.hpp"
#include "aimdk_msgs/srv/set_mc_motion.hpp"
#include "mc_action_switcher.hpp"
#include "rclcpp/rclcpp.hpp"

#include <chrono>
#include <cstdint>
#include <memory>
#include <signal.h>
#include <string>
#include <thread>

constexpr double kServiceCallTimeoutSec = 2.0;
constexpr int kMaxRetryCount = 3;

std::shared_ptr<rclcpp::Node> g_node = nullptr;

void signal_handler(int signal) {
  if (g_node) {
    RCLCPP_INFO(g_node->get_logger(), "Received signal %d, shutting down...",
                signal);
    g_node.reset();
  }
  rclcpp::shutdown();
  exit(signal);
}

class SetMcActionClient : public rclcpp::Node {
public:
  SetMcActionClient() : Node("set_mc_action_client") {
    type_ = this->declare_parameter<std::string>("type", "");
    action_desc_ = this->declare_parameter<std::string>("action_desc", "");
    motion_ = this->declare_parameter<std::string>("motion", "");

    set_motion_client_ = this->create_client<aimdk_msgs::srv::SetMcMotion>(
        "/aimdk_5Fmsgs/srv/SetMcMotion");
    get_client_ = this->create_client<aimdk_msgs::srv::GetMcAction>(
        "/aimdk_5Fmsgs/srv/GetMcAction");
    RCLCPP_INFO(this->get_logger(),
                "SetMcAction client node created with type=%s action_desc=%s "
                "motion=%s",
                type_.c_str(), action_desc_.c_str(), motion_.c_str());
  }

  bool execute() {
    if (!validate_parameters()) {
      return false;
    }

    wait_for_services();

    if (type_ == "action") {
      // Execute action only once (no loop)
      ActionInfo current;
      if (!get_action_status(current)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to get current action status");
        return false;
      }

      RCLCPP_INFO(this->get_logger(), "Current Action is: %s",
                  current.action_desc.c_str());

      std::string target_action;
      if (action_desc_.empty()) {
        std::cout << "\nCurrent Action is: " << current.action_desc
                  << ", please input the expected Action according to the "
                     "motion control state machine transition "
                     "logic in the interface documentation. "
                     "The Action you need to switch: "
                  << std::flush;

        if (!std::getline(std::cin, target_action) || target_action.empty()) {
          RCLCPP_ERROR(this->get_logger(), "No target action specified");
          return false;
        }
      } else {
        target_action = action_desc_;
      }

      aimdk_examples::McActionSwitcher switcher(this->shared_from_this());
      aimdk_examples::McActionSwitchOptions options;
      options.source = "set_mc_action";
      options.total_timeout = std::chrono::seconds(20);

      const auto result = switcher.switch_to(target_action, options);
      if (result.success) {
        std::cout << "\nSwitch succeeded!" << std::endl;
      } else {
        std::cout << "Switch failed, please confirm if the expected Action "
                     "complies with the state machine transition logic"
                  << std::endl;
        RCLCPP_ERROR(this->get_logger(), "Action switch failed: %s",
                     result.message.c_str());
      }

      return result.success;
    } else if (type_ == "motion") {
      // Optimized logic for 'motion' type: Ensure robot is in BIPED_WALK_RUN
      aimdk_examples::McActionSwitcher switcher(this->shared_from_this());
      aimdk_examples::McActionSwitchOptions options;
      options.source = "set_mc_action";
      options.total_timeout = std::chrono::seconds(30);

      const auto result = switcher.switch_to("BIPED_WALK_RUN", options);
      if (!result.success) {
        RCLCPP_ERROR(this->get_logger(),
                     "Failed to switch from %s to BIPED_WALK_RUN: %s",
                     result.current_action.empty() ? "(unknown)"
                                                   : result.current_action.c_str(),
                     result.message.c_str());
        return false;
      }

      // Execute final target motion
      if (!set_motion(motion_)) {
        return false;
      }
      return wait_for_motion();
    }

    RCLCPP_ERROR(this->get_logger(), "Unsupported control type: %s",
                 type_.c_str());
    return false;
  }

private:
  struct ActionInfo {
    int32_t action_id = 0;
    std::string action_desc;
    int32_t status = aimdk_msgs::msg::McActionStatus::IDLE;
  };

  bool validate_parameters() {
    if (type_.empty()) {
      RCLCPP_ERROR(this->get_logger(),
                   "Parameter 'type' must be set. Use 'action' or 'motion'.");
      return false;
    }

    if (type_ != "action" && type_ != "motion") {
      RCLCPP_ERROR(this->get_logger(),
                   "Invalid parameter 'type': %s. Use 'action' or 'motion'.",
                   type_.c_str());
      return false;
    }

    if (type_ == "motion" && motion_.empty()) {
      RCLCPP_ERROR(this->get_logger(),
                   "Parameter 'motion' must be set when type=motion.");
      return false;
    }

    if (type_ == "motion" && motion_.empty()) {
      RCLCPP_ERROR(this->get_logger(),
                   "Parameter 'motion' must be set when type=motion.");
      return false;
    }

    return true;
  }

  // Generic retry function for service calls
  template <typename ServiceT>
  typename rclcpp::Client<ServiceT>::SharedFuture
  call_service_with_retry(
      typename rclcpp::Client<ServiceT>::SharedPtr client,
      typename ServiceT::Request::SharedPtr request,
      const std::string &service_name,
      std::chrono::milliseconds timeout = std::chrono::milliseconds(
          static_cast<int>(kServiceCallTimeoutSec * 1000)),
      int max_retries = kMaxRetryCount) {
    
    for (int i = 0; i < max_retries; ++i) {
      auto future = client->async_send_request(request);
      auto retcode = rclcpp::spin_until_future_complete(
          this->shared_from_this(), future, timeout);

      if (retcode == rclcpp::FutureReturnCode::SUCCESS) {
        return future.future.share();
      }

      RCLCPP_INFO(this->get_logger(),
                  "%s attempt %d/%d timed out, retrying...",
                  service_name.c_str(), i + 1, max_retries);
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    RCLCPP_ERROR(this->get_logger(),
                 "%s failed after %d attempts",
                 service_name.c_str(), max_retries);
    return typename rclcpp::Client<ServiceT>::SharedFuture();
  }

  bool set_motion(const std::string &motion_name) {
    try {
      constexpr int kMaxAttempts = 5;
      const auto request_timeout = std::chrono::seconds(1);

      for (int attempt = 1; attempt <= kMaxAttempts; ++attempt) {
        auto request =
            std::make_shared<aimdk_msgs::srv::SetMcMotion::Request>();
        request->header.stamp = this->now();
        request->source = "set_mc_action";
        request->motion = motion_name;
        request->type = aimdk_msgs::srv::SetMcMotion::Request::MIMIC_QY;
        request->interrupt = false;

        RCLCPP_INFO(this->get_logger(),
                    "Sending SetMcMotion request (%d/%d): motion=%s "
                    "interrupt=false",
                    attempt, kMaxAttempts, motion_name.c_str());

        auto future = set_motion_client_->async_send_request(request);
        auto retcode = rclcpp::spin_until_future_complete(
            shared_from_this(), future, request_timeout);

        if (retcode != rclcpp::FutureReturnCode::SUCCESS) {
          RCLCPP_WARN(this->get_logger(),
                      "SetMcMotion request attempt %d/%d failed or timed out.",
                      attempt, kMaxAttempts);
          continue;
        }

        auto response = future.get();
        const auto code = response->response.header.code;
        const auto state = response->response.state.value;

        if (code == 0 &&
            (state == aimdk_msgs::msg::CommonState::SUCCESS ||
             state == aimdk_msgs::msg::CommonState::RUNNING)) {
          RCLCPP_INFO(this->get_logger(),
                      "SetMcMotion request accepted by service: code=%ld "
                      "state=%d",
                      static_cast<long>(code), state);
          return true;
        }

        RCLCPP_WARN(this->get_logger(),
                    "SetMcMotion request attempt %d/%d was not accepted: "
                    "code=%ld state=%d",
                    attempt, kMaxAttempts, static_cast<long>(code), state);
      }

      RCLCPP_ERROR(this->get_logger(),
                   "Failed to set motion after %d attempts.", kMaxAttempts);
      return false;
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Exception occurred: %s", e.what());
      return false;
    }
  }

  bool wait_for_motion(
      std::chrono::seconds timeout = std::chrono::seconds(10),
      std::chrono::milliseconds poll_interval = std::chrono::milliseconds(200)) {
    auto deadline = std::chrono::steady_clock::now() + timeout;

    RCLCPP_INFO(this->get_logger(),
                "Waiting for current motion action to reach RUNNING state...");

    while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
      ActionInfo info;
      if (!get_action_status(info)) {
        std::this_thread::sleep_for(poll_interval);
        continue;
      }

      if (info.status == aimdk_msgs::msg::McActionStatus::RUNNING) {
        RCLCPP_INFO(this->get_logger(),
                    "Current motion action is running: action_id=%d "
                    "action_desc=%s status=%d",
                    info.action_id, info.action_desc.c_str(), info.status);
        return true;
      }

      std::this_thread::sleep_for(poll_interval);
    }

    RCLCPP_ERROR(this->get_logger(),
                 "Timed out waiting for current motion action to reach "
                 "RUNNING state.");
    return false;
  }

  bool get_action_status(ActionInfo &info) {
    try {
      auto request = std::make_shared<aimdk_msgs::srv::GetMcAction::Request>();
      request->request = aimdk_msgs::msg::CommonRequest();
      request->request.header.stamp = this->now();

      auto future = call_service_with_retry<aimdk_msgs::srv::GetMcAction>(
          get_client_, request, "GetMcAction");

      if (!future.valid()) {
        RCLCPP_WARN(this->get_logger(),
                    "Get current action request service call failed or timed "
                    "out.");
        return false;
      }

      auto response = future.get();
      info.action_id = response->info.current_action.value;
      info.action_desc = response->info.action_desc;
      info.status = response->info.status.value;
      return true;
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Exception occurred: %s", e.what());
      return false;
    }
  }

  template <typename ClientT>
  void wait_for_service(
      const std::shared_ptr<ClientT> &client,
      const std::string &service_name) {
    while (!client->wait_for_service(std::chrono::seconds(2))) {
      if (!rclcpp::ok()) {
        return;
      }
      RCLCPP_INFO(this->get_logger(), "Service unavailable, waiting: %s",
                  service_name.c_str());
    }
    RCLCPP_INFO(this->get_logger(), "Service available: %s",
                service_name.c_str());
  }

  void wait_for_services() {
    /* Check only the necessary services based on the operation type. */
    // GetMcAction is required for both types to query the current state
    wait_for_service(get_client_, "/aimdk_5Fmsgs/srv/GetMcAction");

    if (type_ == "motion") {
      wait_for_service(set_motion_client_, "/aimdk_5Fmsgs/srv/SetMcMotion");
    }
  }

  std::string type_;
  std::string action_desc_;
  std::string motion_;

  rclcpp::Client<aimdk_msgs::srv::SetMcMotion>::SharedPtr set_motion_client_;
  rclcpp::Client<aimdk_msgs::srv::GetMcAction>::SharedPtr get_client_;
};

int main(int argc, char *argv[]) {
  try {
    rclcpp::init(argc, argv);
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    g_node = std::make_shared<SetMcActionClient>();
    auto client = std::dynamic_pointer_cast<SetMcActionClient>(g_node);
    const bool ok = client ? client->execute() : false;

    g_node.reset();
    rclcpp::shutdown();

    return ok ? 0 : 1;
  } catch (const std::exception &e) {
    RCLCPP_ERROR(rclcpp::get_logger("main"),
                 "Program exited with exception: %s", e.what());
    return 1;
  }
}
