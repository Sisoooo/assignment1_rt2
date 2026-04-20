#include <memory>
#include <iostream>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("movement_ui");
  auto goal_pub = node->create_publisher<geometry_msgs::msg::Pose2D>("goal_input", 10);

  // Spin in a background thread so callbacks are processed while we prompt
  std::thread spin_thread([&node]() {
    rclcpp::spin(node);
  });

  while (rclcpp::ok()) {
    double desired_x, desired_y, desired_theta;
    std::cout << "\n--- New Goal ---\n";
    std::cout << "Enter desired x position: ";
    if (!(std::cin >> desired_x)) break;
    std::cout << "Enter desired y position: ";
    if (!(std::cin >> desired_y)) break;
    std::cout << "Enter desired theta (orientation in radians): ";
    if (!(std::cin >> desired_theta)) break;

    auto msg = geometry_msgs::msg::Pose2D();
    msg.x = desired_x;
    msg.y = desired_y;
    msg.theta = desired_theta;
    goal_pub->publish(msg);

    RCLCPP_INFO(node->get_logger(), "Published goal: x = %.2f, y = %.2f, theta = %.2f",
      desired_x, desired_y, desired_theta);
  }

  rclcpp::shutdown();
  spin_thread.join();
  return 0;
}
