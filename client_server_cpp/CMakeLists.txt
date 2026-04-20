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
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rclcpp_components REQUIRED)

add_library(movement_server SHARED src/movement_server.cpp)
add_library(movement_client SHARED src/movement_client.cpp)

ament_target_dependencies(movement_server
  rclcpp
  rclcpp_action
  rclcpp_components
  geometry_msgs
  nav_msgs
  actions_env
  tf2
  tf2_ros
)

ament_target_dependencies(movement_client
  rclcpp
  rclcpp_action
  rclcpp_components
  actions_env
  geometry_msgs
  tf2
  tf2_ros
)

add_executable(movement_ui src/movement_ui.cpp)
ament_target_dependencies(movement_ui
  rclcpp
  geometry_msgs
)

rclcpp_components_register_node(movement_server
  PLUGIN "client_server_cpp::MovementServer"
  EXECUTABLE movement_server_node
)
rclcpp_components_register_node(movement_client
  PLUGIN "client_server_cpp::MovementClient"
  EXECUTABLE movement_client_node
)

ament_export_targets(export_${PROJECT_NAME}_movement_server)
ament_export_targets(export_${PROJECT_NAME}_movement_client)

install(TARGETS movement_server
  EXPORT export_${PROJECT_NAME}_movement_server
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS movement_client
  EXPORT export_${PROJECT_NAME}_movement_client
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS
  movement_ui
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
