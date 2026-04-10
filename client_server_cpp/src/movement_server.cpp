#include <functional>
#include <memory>
#include <thread>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "actions_env/action/movement.hpp"

class MovementServer : public rclcpp::Node
{
public:
  using Movement = actions_env::action::Movement;
  using GoalHandleMovement = rclcpp_action::ServerGoalHandle<Movement>;

  MovementServer()
  : Node("movement_server"), current_x_(0.0)
  {
    auto cb_group = this->create_callback_group(
      rclcpp::CallbackGroupType::Reentrant);

    rcl_action_server_options_t server_options = rcl_action_server_get_default_options();

    action_server_ = rclcpp_action::create_server<Movement>(
      this,
      "movement",
      std::bind(&MovementServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&MovementServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&MovementServer::handle_accepted, this, std::placeholders::_1),
      server_options,
      cb_group);

    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = cb_group;
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom", 10,
      std::bind(&MovementServer::odom_callback, this, std::placeholders::_1),
      sub_options);
  }

private:
  rclcpp_action::Server<Movement>::SharedPtr action_server_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  double current_x_;

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    current_x_ = msg->pose.pose.position.x;
  }

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & /*uuid*/,
    std::shared_ptr<const Movement::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Goal received: x = %.2f", goal->desired_position);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleMovement> /*goal_handle*/)
  {
    RCLCPP_INFO(this->get_logger(), "Cancel request received");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleMovement> goal_handle)
  {
    std::thread{std::bind(&MovementServer::execute, this, std::placeholders::_1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleMovement> goal_handle)
  {
    double desired_x = goal_handle->get_goal()->desired_position;
    auto feedback = std::make_shared<Movement::Feedback>();
    auto cmd = geometry_msgs::msg::Twist();
    rclcpp::Rate rate(10);

    while (rclcpp::ok()) {
      if (goal_handle->is_canceling()) {
        RCLCPP_WARN(this->get_logger(), "Cancel requested, stopping execution");
        cmd.linear.x = 0.0;
        cmd_pub_->publish(cmd);

        auto result = std::make_shared<Movement::Result>();
        result->result = current_x_;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled at x = %.2f", current_x_);
        return;
      }

      double remaining = desired_x - current_x_;
      feedback->remaining = remaining;
      goal_handle->publish_feedback(feedback);

      if (std::abs(remaining) < 0.1) {
        break;
      }

      // Proportional speed, clamped to [-1.0, 1.0]
      double speed = std::max(-1.0, std::min(1.0, 0.5 * remaining));
      cmd.linear.x = speed;
      cmd_pub_->publish(cmd);

      rate.sleep();
    }

    // Stop the robot
    cmd.linear.x = 0.0;
    cmd_pub_->publish(cmd);

    auto result = std::make_shared<Movement::Result>();
    result->result = current_x_;
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Goal reached at x = %.2f", current_x_);
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MovementServer>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
