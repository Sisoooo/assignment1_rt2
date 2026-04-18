#include <functional>
#include <memory>
#include <thread>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp" 
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "actions_env/action/movement.hpp"
#include "tf2/exceptions.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2/LinearMath/Matrix3x3.hpp"
#include "tf2_ros/transform_listener.hpp"
#include "tf2_ros/transform_broadcaster.hpp"
#include "tf2_ros/buffer.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

class MovementServer : public rclcpp::Node
{
public:
  using Movement = actions_env::action::Movement;
  using GoalHandleMovement = rclcpp_action::ServerGoalHandle<Movement>;

  MovementServer()
  : Node("movement_server"), current_x_(0.0), current_y_(0.0), current_theta_(0.0)
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

    // Initialize tf2 buffer, listener, and broadcaster
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // Keep odom subscription for reporting current pose in results/cancellation
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
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  double current_x_;
  double current_y_;
  double current_theta_;

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    current_x_ = msg->pose.pose.position.x;
    current_y_ = msg->pose.pose.position.y;
    double siny_cosp = 2.0 * (msg->pose.pose.orientation.w * msg->pose.pose.orientation.z + msg->pose.pose.orientation.x * msg->pose.pose.orientation.y);
    double cosy_cosp = 1.0 - 2.0 * (msg->pose.pose.orientation.y * msg->pose.pose.orientation.y + msg->pose.pose.orientation.z * msg->pose.pose.orientation.z);
    current_theta_ = std::atan2(siny_cosp, cosy_cosp);

    // Broadcast odom -> base_footprint transform to connect the TF trees
    geometry_msgs::msg::TransformStamped odom_tf;
    odom_tf.header.stamp = msg->header.stamp;
    odom_tf.header.frame_id = "odom";
    odom_tf.child_frame_id = "base_footprint";
    odom_tf.transform.translation.x = msg->pose.pose.position.x;
    odom_tf.transform.translation.y = msg->pose.pose.position.y;
    odom_tf.transform.translation.z = msg->pose.pose.position.z;
    odom_tf.transform.rotation = msg->pose.pose.orientation;
    tf_broadcaster_->sendTransform(odom_tf);
  }

  // Look up the transform from base_link to goal_frame using tf2.
  // Returns true on success, filling dx, dy, distance, and angle_to_goal.
  bool lookup_goal_transform(double & dx, double & dy, double & distance, double & angle_to_goal)
  {
    geometry_msgs::msg::TransformStamped t;
    try {
      t = tf_buffer_->lookupTransform("base_link", "goal_frame", tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(this->get_logger(), "Could not get transform to goal_frame: %s", ex.what());
      return false;
    }
    dx = t.transform.translation.x;
    dy = t.transform.translation.y;
    distance = std::sqrt(dx * dx + dy * dy);
    angle_to_goal = std::atan2(dy, dx);
    return true;
  }

  // Get the remaining orientation error between current yaw and desired theta.
  double get_remaining_yaw(double desired_theta)
  {
    return normalize_angle(desired_theta - current_theta_);
  }

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & /*uuid*/,
    std::shared_ptr<const Movement::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Goal received: x = %.2f, y = %.2f, theta = %.2f", 
    goal->desired_x, goal->desired_y, goal->desired_theta);
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

  double normalize_angle(double angle)
  {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
  }

  void stop_robot()
  {
    auto cmd = geometry_msgs::msg::Twist();
    cmd.linear.x = 0.0;
    cmd.angular.z = 0.0;
    cmd_pub_->publish(cmd);
  }

  bool check_cancel(const std::shared_ptr<GoalHandleMovement> goal_handle)
  {
    if (goal_handle->is_canceling()) {
      RCLCPP_WARN(this->get_logger(), "Cancel requested, stopping execution");
      stop_robot();

      auto result = std::make_shared<Movement::Result>();
      result->result_x = current_x_;
      result->result_y = current_y_;
      result->result_theta = current_theta_;
      goal_handle->canceled(result);
      RCLCPP_INFO(this->get_logger(), "Goal canceled at x = %.2f, y = %.2f, theta = %.2f",
        current_x_, current_y_, current_theta_);
      return true;
    }
    return false;
  }

  void execute(const std::shared_ptr<GoalHandleMovement> goal_handle)
  {
    double desired_theta = goal_handle->get_goal()->desired_theta;

    auto feedback = std::make_shared<Movement::Feedback>();
    auto cmd = geometry_msgs::msg::Twist();
    rclcpp::Rate rate(10);

    // Phase 1: Drive toward the target position using tf2 lookup (blended rotation + translation)
    while (rclcpp::ok()) {
      if (check_cancel(goal_handle)) return;

      double dx, dy, distance, angle_to_goal;
      if (!lookup_goal_transform(dx, dy, distance, angle_to_goal)) {
        rate.sleep();
        continue;
      }

      feedback->remaining_distance = distance;
      feedback->remaining_angle = std::abs(get_remaining_yaw(desired_theta));
      goal_handle->publish_feedback(feedback);

      if (distance < 0.1) break;

      // Scale linear speed down when heading error is large, so the robot
      // curves toward the goal rather than driving then turning
      double heading_factor = std::cos(angle_to_goal);  // 1 when aligned, 0 at 90°, -1 when backwards
      double speed = 0.5 * distance * std::max(0.0, heading_factor);
      speed = std::min(speed, 1.0);

      cmd.linear.x = speed;
      cmd.angular.z = std::max(-1.0, std::min(1.0, 1.5 * angle_to_goal));
      cmd_pub_->publish(cmd);
      rate.sleep();
    }

    // Phase 2: Rotate to the desired final orientation
    while (rclcpp::ok()) {
      if (check_cancel(goal_handle)) return;

      double angle_error = get_remaining_yaw(desired_theta);

      feedback->remaining_distance = 0.0;
      feedback->remaining_angle = std::abs(angle_error);
      goal_handle->publish_feedback(feedback);

      if (std::abs(angle_error) < 0.05) break;

      cmd.linear.x = 0.0;
      cmd.angular.z = std::max(-1.0, std::min(1.0, 1.5 * angle_error));
      cmd_pub_->publish(cmd);
      rate.sleep();
    }

    // Stop the robot
    stop_robot();

    auto result = std::make_shared<Movement::Result>();
    result->result_x = current_x_;
    result->result_y = current_y_;
    result->result_theta = current_theta_;
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Goal reached at x = %.2f, y = %.2f, theta = %.2f",
      current_x_, current_y_, current_theta_);
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
