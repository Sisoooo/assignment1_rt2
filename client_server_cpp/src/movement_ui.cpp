#include "movement_client.cpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto client = std::make_shared<MovementClient>();

  double desired_x;
  std::cout << "Enter desired x position: ";
  std::cin >> desired_x;
  double desired_y;
  std::cout << "Enter desired y position: ";
  std::cin >> desired_y;
  double desired_theta;
  std::cout << "Enter desired theta (orientation in radians): ";
  std::cin >> desired_theta;

  client->send_goal(desired_x, desired_y, desired_theta);

  rclcpp::spin(client);
  return 0;
}
