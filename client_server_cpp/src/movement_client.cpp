#include <functional>
#include <future>
#include <memory>
#include <string>
#include <iostream>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "actions_env/action/movement.hpp"

class MovementClient : public rclcpp::Node
{
public:
  using Movement = actions_env::action::Movement;
  using GoalHandleMovement = rclcpp_action::ClientGoalHandle<Movement>;

  MovementClient()
  : Node("movement_client"), cancel_sent_(false)
  {
    action_client_ = rclcpp_action::create_client<Movement>(this, "movement");
  }

  void send_goal(double desired_x, double desired_y, double desired_theta)
  {
    desired_x_ = desired_x;
    desired_y_ = desired_y;
    desired_theta_ = desired_theta;
    cancel_sent_ = false;
    goal_handle_ = nullptr;

    RCLCPP_INFO(this->get_logger(), "Waiting for action server...");
    action_client_->wait_for_action_server();

    auto goal_msg = Movement::Goal();
    goal_msg.desired_x = desired_x;
    goal_msg.desired_y = desired_y;
    goal_msg.desired_theta = desired_theta;

    RCLCPP_INFO(this->get_logger(), "Sending goal: x = %.2f, y = %.2f, theta = %.2f", desired_x, desired_y, desired_theta);

    auto send_goal_options = rclcpp_action::Client<Movement>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      std::bind(&MovementClient::goal_response_callback, this, std::placeholders::_1);
    send_goal_options.feedback_callback =
      std::bind(&MovementClient::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
    send_goal_options.result_callback =
      std::bind(&MovementClient::result_callback, this, std::placeholders::_1);

    action_client_->async_send_goal(goal_msg, send_goal_options);
  }

private:
  rclcpp_action::Client<Movement>::SharedPtr action_client_;
  GoalHandleMovement::SharedPtr goal_handle_;
  double desired_x_;
  double desired_y_;
  double desired_theta_;
  bool cancel_sent_;

  void goal_response_callback(const GoalHandleMovement::SharedPtr & goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_INFO(this->get_logger(), "Goal rejected");
      return;
    }
    goal_handle_ = goal_handle;
    RCLCPP_INFO(this->get_logger(), "Goal accepted");
  }

  void feedback_callback(
    GoalHandleMovement::SharedPtr /*goal_handle*/,
    const std::shared_ptr<const Movement::Feedback> feedback)
  {
    RCLCPP_INFO(this->get_logger(), "Remaining distance: %.2f, angle: %.2f", feedback->remaining_distance, feedback->remaining_angle);

    if (cancel_sent_ || !goal_handle_) {
      return;
    }

    if ((desired_x_ > 10.0 || desired_x_ < -10.0) || (desired_y_ > 10.0 || desired_y_ < -10.0)) {
      cancel_sent_ = true;
      RCLCPP_WARN(this->get_logger(), "Goal position is outside the given environment, cancelling goal...");
      auto cancel_future = action_client_->async_cancel_goal(goal_handle_);
    }
  }

  void result_callback(const GoalHandleMovement::WrappedResult & result)
  {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(this->get_logger(), "Final position: x = %.2f, y = %.2f, theta = %.2f", 
        result.result->result_x, result.result->result_y, result.result->result_theta);
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_WARN(this->get_logger(), "Goal aborted");
        break;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_INFO(this->get_logger(), "Goal canceled at x = %.2f, y = %.2f, theta = %.2f", 
        result.result->result_x, result.result->result_y, result.result->result_theta);
        break;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        break;
    }
    rclcpp::shutdown();
  }
};
