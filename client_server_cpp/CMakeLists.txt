cmake_minimum_required(VERSION 3.8)
project(client_server_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(actions_env REQUIRED)

add_executable(movement_server src/movement_server.cpp)
ament_target_dependencies(movement_server
  rclcpp
  rclcpp_action
  geometry_msgs
  nav_msgs
  actions_env
)

add_executable(movement_ui src/movement_ui.cpp)
ament_target_dependencies(movement_ui
  rclcpp
  rclcpp_action
  actions_env
)

install(TARGETS
  movement_server
  movement_ui
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
