// Copyright 2026 PrimeBot Team
//
// Header-only Q1 MC action switcher.  It mirrors action_ruler.yaml in the
// SDK, plans a route from the current action, and waits for every requested
// MC runner transition before it continues to the final target.

#ifndef AIMDK_EXAMPLES_CPP_MC_ACTION_SWITCHER_HPP_
#define AIMDK_EXAMPLES_CPP_MC_ACTION_SWITCHER_HPP_

#include "aimdk_msgs/msg/common_request.hpp"
#include "aimdk_msgs/msg/common_state.hpp"
#include "aimdk_msgs/msg/mc_action_command.hpp"
#include "aimdk_msgs/msg/mc_action_status.hpp"
#include "aimdk_msgs/srv/get_mc_action.hpp"
#include "aimdk_msgs/srv/set_mc_action.hpp"
#include "rclcpp/rclcpp.hpp"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <deque>
#include <memory>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace aimdk_examples
{

struct McActionSwitchOptions
{
  std::string source = "sdk_node";
  std::chrono::milliseconds service_wait_timeout{5000};
  std::chrono::milliseconds service_call_timeout{3000};
  std::chrono::milliseconds poll_interval{500};
  std::chrono::milliseconds minimum_set_action_interval{500};
  std::chrono::milliseconds total_timeout{20000};
};

enum class McActionSwitchError
{
  kNone,
  kInvalidArgument,
  kServiceUnavailable,
  kGetActionFailed,
  kTargetNotActive,
  kNoTransitionPath,
  kSetActionFailed,
  kTimeout,
  kShutdown,
};

struct McActionSwitchResult
{
  bool success = false;
  McActionSwitchError error = McActionSwitchError::kNone;
  std::string current_action;
  std::string target_action;
  std::string message;
};

// Switches to target_action synchronously.  The target and all intermediate
// action names use the action_desc strings from q1_t2d0/action_ruler.yaml.
class McActionSwitcher
{
 public:
  explicit McActionSwitcher(
      const rclcpp::Node::SharedPtr & node,
      const std::string & set_action_service =
          "/aimdk_5Fmsgs/srv/SetMcAction",
      const std::string & get_action_service =
          "/aimdk_5Fmsgs/srv/GetMcAction")
  : node_(node),
    set_action_service_(set_action_service),
    get_action_service_(get_action_service)
  {
    if (node_) {
      set_action_client_ =
          node_->create_client<aimdk_msgs::srv::SetMcAction>(set_action_service_);
      get_action_client_ =
          node_->create_client<aimdk_msgs::srv::GetMcAction>(get_action_service_);
    }
  }

  McActionSwitchResult switch_to(
      const std::string & target_action,
      const McActionSwitchOptions & options = McActionSwitchOptions())
  {
    McActionSwitchResult result;
    result.target_action = target_action;

    if (!node_ || target_action.empty()) {
      return fail(result, McActionSwitchError::kInvalidArgument,
                  "Node and target_action must be provided.");
    }
    if (!wait_for_services(options, result)) {
      return result;
    }

    ActionState current;
    if (!get_current_action(options, current)) {
      return fail(result, McActionSwitchError::kGetActionFailed,
                  "GetMcAction did not return the current action.");
    }
    result.current_action = current.action_desc;
    if (current.action_desc == target_action) {
      if (is_active(current)) {
        result.success = true;
        result.message = "Robot is already in the requested action.";
        return result;
      }
      return fail(result, McActionSwitchError::kTargetNotActive,
                  "MC reports the target action as IDLE; refusing to leave " +
                      target_action + " through a fallback transition.");
    }

    const std::vector<PlannedTransition> path =
        plan_path(current.action_desc, target_action);
    if (path.empty()) {
      return fail(result, McActionSwitchError::kNoTransitionPath,
                  "No Q1 action_ruler path from " + current.action_desc +
                      " to " + target_action + ".");
    }

    const auto deadline = std::chrono::steady_clock::now() + options.total_timeout;
    auto next_request_time = std::chrono::steady_clock::now();
    for (const auto & transition : path) {
      if (!rclcpp::ok()) {
        return fail(result, McActionSwitchError::kShutdown,
                    "ROS shutdown while switching actions.");
      }
      if (std::chrono::steady_clock::now() >= deadline) {
        return fail(result, McActionSwitchError::kTimeout,
                    "Timed out while switching actions.");
      }

      std::this_thread::sleep_until(next_request_time);
      if (!request_action(transition.command, options)) {
        return fail(result, McActionSwitchError::kSetActionFailed,
                    "SetMcAction rejected " + transition.command + ".");
      }
      next_request_time = std::chrono::steady_clock::now() +
                          options.minimum_set_action_interval;

      ActionState reached;
      if (!wait_for_action(transition.expected_action, options, deadline,
                           reached)) {
        if (!rclcpp::ok()) {
          return fail(result, McActionSwitchError::kShutdown,
                      "ROS shutdown while waiting for " +
                          transition.expected_action + ".");
        }
        return fail(result, McActionSwitchError::kTimeout,
                    "Timed out waiting for " + transition.expected_action +
                        " after requesting " + transition.command + ".");
      }
      current = reached;
      result.current_action = current.action_desc;
    }

    if (current.action_desc != target_action) {
      return fail(result, McActionSwitchError::kTimeout,
                  "MC did not reach the requested target action.");
    }
    result.success = true;
    result.message = "Robot reached the requested action.";
    return result;
  }

 private:
  struct ActionState
  {
    std::string action_desc;
    int32_t status = aimdk_msgs::msg::McActionStatus::IDLE;
  };

  struct PlannedTransition
  {
    std::string command;
    std::string expected_action;
  };

  static McActionSwitchResult & fail(
      McActionSwitchResult & result, McActionSwitchError error,
      const std::string & message)
  {
    result.success = false;
    result.error = error;
    result.message = message;
    return result;
  }

  static bool is_active(const ActionState & action)
  {
    return action.status != aimdk_msgs::msg::McActionStatus::IDLE;
  }

  static const std::unordered_map<std::string, std::vector<std::string>> &
  action_rules()
  {
    static const std::unordered_map<std::string, std::vector<std::string>> rules = {
        {"PASSIVE_DEFAULT", {"BIPED_STAND_DEFAULT", "LYING_DEFAULT",
                               "STAND_UP", "STORE_DEFAULT", "BIPED_GROUND_POSE",
                               "LIE_FACE_DOWN", "LIE_FACE_UP",
                               "BIPED_LEAVE_SEAT"}},
        {"DAMPING_DEFAULT", {"PASSIVE_DEFAULT"}},
        {"BIPED_STAND_DEFAULT", {"BIPED_WALK_RUN", "BIPED_WALK_RUN_DEFAULT",
                                  "STORE_DEFAULT", "BIPED_BLIND_TERRAIN"}},
        {"BIPED_WHOLE_BODY_CTRL", {"BIPED_STAND_DEFAULT", "BIPED_WALK_RUN",
                                     "BIPED_WALK_RUN_DEFAULT",
                                     "BIPED_BLIND_TERRAIN", "LIE_FACE_UP",
                                     "LIE_FACE_DOWN", "STORE_DEFAULT",
                                     "BIPED_RECORD_UPPER"}},
        {"BIPED_CUSTOM_UPPER", {"BIPED_STAND_DEFAULT", "BIPED_WALK_RUN"}},
        {"STORE_DEFAULT", {"BIPED_GROUND_POSE", "LIE_FACE_DOWN", "LIE_FACE_UP"}},
        {"BIPED_WALK_RUN", {"BIPED_WHOLE_BODY_CTRL",
                              "BIPED_STAND_DEFAULT", "BIPED_WALK_RUN_DEFAULT",
                              "LIE_FACE_UP", "LIE_FACE_DOWN", "BIPED_BLIND_TERRAIN",
                              "STORE_DEFAULT", "BIPED_RECORD_UPPER",
                              "BIPED_CUSTOM_UPPER",
                              "BIPED_TAKE_SEAT", "BIPED_RUN"}},
        {"BIPED_WALK_RUN_DEFAULT", {"BIPED_STAND_DEFAULT",
                                     "BIPED_WHOLE_BODY_CTRL", "BIPED_WALK_RUN"}},
        {"BIPED_BLIND_TERRAIN", {"BIPED_WALK_RUN"}},
        {"LYING_DEFAULT", {"STORE_DEFAULT"}},
        {"LIE_FACE_UP", {"LYING_DEFAULT"}},
        {"LIE_FACE_DOWN", {"LYING_DEFAULT"}},
        {"STAND_UP", {"BIPED_WALK_RUN", "BIPED_BLIND_TERRAIN"}},
        {"BIPED_GROUND_POSE", {"STAND_UP", "STORE_DEFAULT"}},
        {"BIPED_RECORD_UPPER", {"BIPED_WHOLE_BODY_CTRL", "BIPED_BLIND_TERRAIN",
                                 "BIPED_WALK_RUN", "BIPED_WALK_RUN_DEFAULT"}},
        {"BIPED_LEAVE_SEAT", {"BIPED_WALK_RUN"}},
        {"BIPED_TAKE_SEAT", {"PASSIVE_DEFAULT"}},
        {"BIPED_RUN", {"BIPED_WALK_RUN"}},
    };
    return rules;
  }

  static const std::unordered_set<std::string> & skip_actions()
  {
    static const std::unordered_set<std::string> actions = {
        "PASSIVE_DEFAULT", "BIPED_WALK_RUN", "BIPED_BLIND_TERRAIN",
        "LIE_FACE_UP", "LIE_FACE_DOWN", "STAND_UP", "STORE_DEFAULT",
        "BIPED_STAND_DEFAULT", "BIPED_WALK_RUN_DEFAULT",
        "BIPED_LEAVE_SEAT", "BIPED_TAKE_SEAT",
    };
    return actions;
  }

  static const std::unordered_map<std::string, std::string> &
  automatic_next_actions()
  {
    // STAND_UP is the Q1 recovery action: its runner updates MC to
    // BIPED_WALK_RUN after standing.  Do not send BIPED_WALK_RUN separately
    // until it has been observed through GetMcAction.
    static const std::unordered_map<std::string, std::string> actions = {
        {"STAND_UP", "BIPED_WALK_RUN"},
    };
    return actions;
  }

  static bool is_bridge_action(const std::string & action)
  {
    return action == "BIPED_WALK_RUN";
  }

  static std::vector<PlannedTransition> plan_path(
      const std::string & current_action, const std::string & target_action)
  {
    struct Previous
    {
      std::string action;
      PlannedTransition transition;
    };

    std::deque<std::string> pending;
    std::unordered_set<std::string> visited;
    std::unordered_map<std::string, Previous> previous;
    pending.push_back(current_action);
    visited.insert(current_action);

    while (!pending.empty()) {
      const std::string from = pending.front();
      pending.pop_front();

      std::vector<std::string> candidates;
      const auto rules_it = action_rules().find(from);
      if (rules_it != action_rules().end()) {
        candidates = rules_it->second;
      }
      
      if (target_action == "PASSIVE_DEFAULT" ||
          target_action == "DAMPING_DEFAULT") {
        candidates.push_back(target_action);
      }

      for (const auto & candidate : candidates) {
        PlannedTransition transition;
        std::string arrival;
        if (candidate == target_action) {
          // A skip action is valid when it is the actual caller target or a
          // direct one-hop command; it is not expanded as a normal bridge.
          transition = {candidate, candidate};
          arrival = candidate;
        } else {
          const auto automatic_it = automatic_next_actions().find(candidate);
          if (automatic_it != automatic_next_actions().end()) {
            transition = {candidate, automatic_it->second};
            arrival = automatic_it->second;
          } else if (from == "DAMPING_DEFAULT" &&
                     candidate == "PASSIVE_DEFAULT") {
            transition = {candidate, candidate};
            arrival = candidate;
          } else if (is_bridge_action(candidate) ||
                     skip_actions().count(candidate) == 0U) {
            transition = {candidate, candidate};
            arrival = candidate;
          } else {
            continue;
          }
        }

        if (!visited.insert(arrival).second) {
          continue;
        }
        previous.emplace(arrival, Previous{from, transition});
        if (arrival == target_action) {
          std::vector<PlannedTransition> path;
          std::string cursor = target_action;
          while (cursor != current_action) {
            const auto previous_it = previous.find(cursor);
            if (previous_it == previous.end()) {
              return {};
            }
            path.push_back(previous_it->second.transition);
            cursor = previous_it->second.action;
          }
          std::reverse(path.begin(), path.end());
          return path;
        }
        pending.push_back(arrival);
      }
    }
    return {};
  }

  bool wait_for_services(const McActionSwitchOptions & options,
                         McActionSwitchResult & result) const
  {
    if (!set_action_client_->wait_for_service(options.service_wait_timeout)) {
      fail(result, McActionSwitchError::kServiceUnavailable,
           "SetMcAction service is unavailable: " + set_action_service_);
      return false;
    }
    if (!get_action_client_->wait_for_service(options.service_wait_timeout)) {
      fail(result, McActionSwitchError::kServiceUnavailable,
           "GetMcAction service is unavailable: " + get_action_service_);
      return false;
    }
    return true;
  }

  bool get_current_action(const McActionSwitchOptions & options,
                          ActionState & action) const
  {
    constexpr int max_retries = 3;
    for (int attempt = 1; attempt <= max_retries; ++attempt) {
      auto request =
          std::make_shared<aimdk_msgs::srv::GetMcAction::Request>();
      request->request = aimdk_msgs::msg::CommonRequest();
      request->request.header.stamp = node_->now();

      auto future = get_action_client_->async_send_request(request);
      const auto return_code = rclcpp::spin_until_future_complete(
          node_, future, options.service_wait_timeout);

      if (return_code == rclcpp::FutureReturnCode::SUCCESS) {
        const auto response = future.get();
        try {
          if (!response || response->header.code != 0) {
            const auto code = response ? std::to_string(response->header.code)
                                       : std::string("null");
            RCLCPP_WARN(node_->get_logger(),
                        "GetMcAction attempt %d/%d returned code=%s.",
                        attempt, max_retries, code.c_str());
          } else if (response->info.action_desc.empty()) {
            RCLCPP_WARN(node_->get_logger(),
                        "GetMcAction attempt %d/%d returned empty action_desc.",
                        attempt, max_retries);
          } else {
            action.action_desc = response->info.action_desc;
            action.status = response->info.status.value;
            return true;
          }
        } catch (const std::exception & e) {
          RCLCPP_WARN(node_->get_logger(),
                      "GetMcAction attempt %d/%d failed: %s.",
                      attempt, max_retries, e.what());
        }
      } else {
        RCLCPP_WARN(
            node_->get_logger(),
            "GetMcAction attempt %d/%d timed out or was interrupted.",
            attempt, max_retries);
      }

      if (attempt < max_retries) {
        std::this_thread::sleep_for(options.poll_interval);
      }
    }
    return false;
  }

  bool request_action(const std::string & action_desc,
                      const McActionSwitchOptions & options) const
  {
    constexpr int max_retries = 3;
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    RCLCPP_INFO(node_->get_logger(), "Requesting MC action: %s",
                action_desc.c_str());

    for (int attempt = 1; attempt <= max_retries; ++attempt) {
      auto request = std::make_shared<aimdk_msgs::srv::SetMcAction::Request>();
      request->header.stamp = node_->now();
      request->source = options.source;
      request->command = aimdk_msgs::msg::McActionCommand();
      request->command.action_desc = action_desc;

      auto future = set_action_client_->async_send_request(request);
      const auto return_code = rclcpp::spin_until_future_complete(
          node_, future, options.service_call_timeout);

      if (return_code != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_WARN(node_->get_logger(),
                    "SetMcAction(%s) attempt %d/%d timed out.",
                    action_desc.c_str(), attempt, max_retries);
        if (attempt < max_retries) {
          std::this_thread::sleep_for(options.poll_interval);
        }
        continue;
      }

      try {
        const auto response = future.get();
        if (response && response->response.header.code == 0 &&
            response->response.state.value == aimdk_msgs::msg::CommonState::SUCCESS) {
          return true;
        }

        // 服务端返回了但状态未成功 → 不重试
        const auto code = response ? std::to_string(response->response.header.code)
                                   : std::string("null");
        RCLCPP_WARN(node_->get_logger(),
                    "SetMcAction(%s) rejected: code=%s.",
                    action_desc.c_str(), code.c_str());
        return false;
      } catch (const std::exception & e) {
        RCLCPP_WARN(node_->get_logger(),
                    "SetMcAction(%s) attempt %d/%d failed: %s.",
                    action_desc.c_str(), attempt, max_retries, e.what());
        if (attempt < max_retries) {
          std::this_thread::sleep_for(options.poll_interval);
        }
      }
    }
    return false;
  }

  bool wait_for_action(const std::string & expected_action,
                       const McActionSwitchOptions & options,
                       const std::chrono::steady_clock::time_point & deadline,
                       ActionState & reached) const
  {
    while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
      ActionState current;
      if (get_current_action(options, current) &&
          current.action_desc == expected_action && is_active(current)) {
        RCLCPP_INFO(node_->get_logger(), "MC reached action: %s (status=%d)",
                    expected_action.c_str(), current.status);
        reached = current;
        return true;
      }
      std::this_thread::sleep_for(options.poll_interval);
    }
    return false;
  }

  rclcpp::Node::SharedPtr node_;
  std::string set_action_service_;
  std::string get_action_service_;
  rclcpp::Client<aimdk_msgs::srv::SetMcAction>::SharedPtr set_action_client_;
  rclcpp::Client<aimdk_msgs::srv::GetMcAction>::SharedPtr get_action_client_;
};

}  // namespace aimdk_examples

#endif  // AIMDK_EXAMPLES_CPP_MC_ACTION_SWITCHER_HPP_
