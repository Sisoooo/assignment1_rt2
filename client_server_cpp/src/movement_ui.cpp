#include "movement_client.cpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto client = std::make_shared<MovementClient>();

  // Spin in a background thread so callbacks are processed while we prompt
  std::thread spin_thread([&client]() {
    rclcpp::spin(client);
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

    client->send_goal(desired_x, desired_y, desired_theta);

    // Wait until the goal completes (succeeded, canceled, or aborted)
    rclcpp::Rate wait_rate(10);
    while (rclcpp::ok() && !client->is_goal_done()) {
      wait_rate.sleep();
    }
  }

  rclcpp::shutdown();
  spin_thread.join();
  return 0;
}
