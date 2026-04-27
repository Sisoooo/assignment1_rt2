# Robot control in Gazebo/Rviz with actions, componenents and frames.

This repository contains code for a Gazebo simulation of a mobile physical robot, with the possibility for the user to move it using an external interface. The interface allows the user to input a certain pose as a series of inputs (x coordinate, y coordinate and orientation theta in radians) which will then be published to the pose2D topic. The action client, which is subscribed to this topic, will then read the given pose and broadcast it as a goal frame using the tf2 package. The action server will then use the *lookupTransform* function to read the goal frame and compute the robot's movement towards that frame, until reaching it. The user interface then proposes a new prompt to input, until exiting manually. 
the user interface also allows the cancellation of a goal frame in two cases: 
- when inserting a coordinate that is outide the simulation environment;
- while the robot is moving, by pressing the 'c' button.
The action client and server are implemented as components of a same container, to speed up inter-process communication.
It is worth mentioning that this model does not implement obstacle avoidance.

## Code explanation and structure
The project is composed of three packages, each containing specific features:

- actions_env, a cpp package the contains the action file (*Movement.action*) that is used by client and server to describe the goal frame;

- client_server_cpp, containing the action client-server architecture (*movement_client.cpp*, *movement_server.cpp*), which is written as two components, and a separate node (*movement_ui.cpp*) for the user interface.

- simulation_env, which provides the Gazebo/Rviz simulation environment. 

## How to run
Requirements: ROS2 setup, Gazebo, Rviz.
Project tested on Docker image tiryoh/ros2-desktop-vnc:jazzy-20251019T1559

Procedure to run:
- Terminal setup (repeat for each terminal)
  ```
  colcon build
  source install/setup.bash
  ```
- Terminal 1 (Gazebo/Rviz)
  ```
  ros2 launch simulation_env spawn_robot_ex.launch.py
  ```
- Terminal 2 (client-server and user interface)
  ```
  ros2 launch client_server_cpp movement.launch.py
  ```

- To better see the frame publication, use Rviz with odom as fixed frame and add the topic TF in the simulation if not already present

## Author 
Sisani Francesco