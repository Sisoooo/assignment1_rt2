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

  void send_goal(double desired_x)
  {
    desired_x_ = desired_x;
    cancel_sent_ = false;
    goal_handle_ = nullptr;

    RCLCPP_INFO(this->get_logger(), "Waiting for action server...");
    action_client_->wait_for_action_server();

    auto goal_msg = Movement::Goal();
    goal_msg.desired_position = desired_x;

    RCLCPP_INFO(this->get_logger(), "Sending goal: x = %.2f", desired_x);

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
    RCLCPP_INFO(this->get_logger(), "Remaining distance: %.2f", feedback->remaining);

    if (cancel_sent_ || !goal_handle_) {
      return;
    }

    if (desired_x_ > 10.0 || desired_x_ < -10.0) {
      cancel_sent_ = true;
      RCLCPP_WARN(this->get_logger(), "Goal position is outside [-10, 10], cancelling goal...");
      auto cancel_future = action_client_->async_cancel_goal(goal_handle_);
    }
  }

  void result_callback(const GoalHandleMovement::WrappedResult & result)
  {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(this->get_logger(), "Final position: x = %.2f", result.result->result);
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_WARN(this->get_logger(), "Goal aborted");
        break;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_INFO(this->get_logger(), "Goal canceled at x = %.2f", result.result->result);
        break;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        break;
    }
    rclcpp::shutdown();
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto client = std::make_shared<MovementClient>();

  double desired_x;
  std::cout << "Enter desired x position: ";
  std::cin >> desired_x;

  client->send_goal(desired_x);

  rclcpp::spin(client);
  return 0;
}
