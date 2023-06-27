# Solar Autobot
## Abstract
*This project presents the design, development, and implementation of a two-wheel differential robot capable of autonomous movement and sunlight-based charging. The robot incorporates light sensors and a solar panel, to optimize solar energy capture and facilitate navigation toward light. The Gazebo simulation environment and ROS2 framework are utilized to ensure testing, evaluation, and control of the robot's performance.
The mechanical structure of the two-wheel differential robot consists of a central chassis connected to independently controlled wheels. The wheels are connected to the chassis, facilitating forward, backward, and turning motions.*

*The robot’s control system developed using the ROS2 framework, processes sensory input from the light sensors, enabling precise movements to align the robot with the sun for optimal solar energy capture.
To harness solar energy efficiently, the robot is equipped with a solar panel mounted on top of its chassis. The solar panel directly interfaces with sunlight, converting the captured solar energy into electrical energy. This energy is stored in a battery, providing a sustainable power source for prolonged operation without the need for external charging or the energy can be used for simple storage and usage.
The integration of the Gazebo simulation environment allows for realistic testing and evaluation of the robot's movements, charging capabilities, and overall performance. Gazebo's physics-based simulations enable the analysis of the robot's behaviour under various lighting conditions and terrains, aiding in algorithm development, performance optimization, and system validation.
The utilization of the ROS2 framework and Gazebo simulation environment further enhances the system's reliability, robustness, and scalability, opening avenues for future advancements in solar-powered autonomous robots.
Keywords: Two-wheel differential robot, Light sensors, Solar panel, Autonomous movement, Sunlight-based charging, Gazebo simulation, ROS2 framework, Sustainable energy.*

## Novelty

*This project aims to simulate a Two-Wheel Autonomous Differential Drive Robot using ROS 2 and Gazebo. The novelty of this project lies in the implementation of a light detecting sensor that enables the robot to move towards the light.*

## Requirements
--> *_ROS2 Humble_*\
--> *_Ubuntu 22.04_*\
--> *_Gazebo Simulation tool_*

## Setting up the package
Step 1: Create a Workspace

```bash
mkdir solar_autobot_ws
cd solar_autobot_ws
```
Step2: Create a source folder
```bash
mkdir src
cd src
```
Step3 : Git Clone the directory
```bash
git clone https://github.com/Tarunesh38/Solar_Autobot_Batch-B_17/Software_Solar_Autobot/src/.git
```
Step4 : Move to workspace folder
```bash
cd ~/solar_autobot_ws
```
Step5 : Colcon build
```bash
colcon build
```
or
```bash
colcon build --packages-select solar_autobot_ws
```
Step6 : Source your workspace
```bash
source ~/solar_autobot_ws/install.setup.bash
```
Step7 : Load your launch file in your workspace
```bash
cd ~/solar_autobot_ws
```
```bash
ros2 launch solar_robot_spawner_pkg gazebo_world.launch.py
```

## Automate Your Robot 
Load Controller Launch file to automate the robot
```bash
ros2 launch solar_robot_controller_pkg controller_estimator.launch.py
```
Let’s see what topics are active. Open a new terminal, and type:
```bash
ros2 topic list -t
```

## Manually move the robot
Open a new terminal:
```bash
ros2 launch solar_robot_spawner_pkg gazebo_world.launch.py
```
Again, Open a new Terminal and Type:
```bash
sudo apt install ros-foxy-turtlebot3*
```
```bash
ros2 run turtlebot3_teleop teleop_keyboard --ros-args --remap /cmd_vel:=/demo/cmd_vel
```

## Simulation
https://github.com/Tarunesh38/Solar_Autobot_Batch-B_17/assets/119646778/e0bdb9b3-3da0-4e1c-961a-e032e239929c
