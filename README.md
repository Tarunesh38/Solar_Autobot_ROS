# Solar Autobot - ROS
## Abstract

*This project presents the design, development, and implementation of a two-wheel differential robot capable of autonomous movement and sunlight-based charging. The robot incorporates light sensors and a solar panel, to optimize solar energy capture and facilitate navigation toward light by sensing the color of the light. The Gazebo simulation environment and ROS2 framework are utilized to ensure testing, evaluation, and control of the robot's performance. The mechanical structure of the two-wheel differential robot consists of a central chassis connected to independently controlled wheels. The wheels are connected to the chassis, facilitating forward, backward, and turning motions.*

*The robot’s control system developed using the ROS2 framework, processes sensory input from the camera using opencv, enabling precise movements to align the robot with the sun for optimal solar energy capture, if we the match the color of the light source of the input and the color observed. To harness solar energy efficiently, the robot is equipped with a solar panel mounted on top of its chassis. The solar panel directly interfaces with sunlight, converting the captured solar energy into electrical energy. This energy is stored in a battery, providing a sustainable power source for prolonged operation without the need for external charging or the energy can be used for simple storage and usage. The integration of the Gazebo simulation environment allows for realistic testing and evaluation of the robot's movements, charging capabilities, and overall performance. Gazebo's physics-based simulations enable the analysis of the robot's behaviour under various lighting conditions and terrains, aiding in algorithm development, performance optimization, and system validation. The utilization of the ROS2 framework and Gazebo simulation environment further enhances the system's reliability, robustness, and scalability, opening avenues for future advancements in solar-powered autonomous robots.*

*Keywords: Two-wheel differential robot, Camera, LIDAR, Colour-Based, Solar panel, Autonomous movement, Sunlight-based charging, Gazebo simulation, ROS2 framework, Sustainable energy.*

## Novelty

*This project aims to simulate a Two-Wheel Autonomous Differential Drive Robot using ROS 2 and Gazebo. The novelty of this project lies in the implementation of a light detecting sensor that enables the robot to move towards the light.*

## Team Members - Batch - B_17
_Parthvi Manoj      - CB.EN.U4AIE21143_\
_Sakthi Swaroopan S - CB.EN.U4AIE21159_\
_Sanjay Chidambaram - CB.EN.U4AIE21160_\
_Taruneshwaran T    - CB.EN.U4AIE21170_

## Requirements
--> *_ROS2 Humble_*\
--> *_Ubuntu 22.04_*\
--> *_Gazebo Simulation tool_*\
--> *_Rviz Vizualization tool_*\
--> *_OpenCV_*

## Setting up the package
Step 1: Create a Workspace

```bash
mkdir solar_autobot_ws
cd solar_autobot_ws
```
Step 2: Create a source folder
```bash
mkdir src
cd src
```
Step 3: Git Clone the directory
```bash
git clone https://github.com/Tarunesh38/Solar_Autobot_Batch-B_17/Software_Solar_Autobot/src/.git
```
Step 4: Move to workspace folder
```bash
cd ~/solar_autobot_ws
```
Step 5: Colcon build
```bash
colcon build
```
or
```bash
colcon build --packages-select solar_autobot_ws
```
Step 6: Source your workspace
```bash
source ~/solar_autobot_ws/install.setup.bash
```
Step 7: Load your launch file in your workspace
```bash
cd ~/solar_autobot_ws
```
```bash
ros2 launch solar_robot_spawner_pkg gazebo_world.launch.py
```
## Spawning robot in the world
![Robot3](https://github.com/Tarunesh38/Solar_Autobot_Batch-B_17/assets/119646778/745b635c-a47c-4f64-99c0-e6e060762fb4)
![Robot4](https://github.com/Tarunesh38/Solar_Autobot_Batch-B_17/assets/119646778/9dee1a7b-3ba9-4d2c-8d1b-719d82d895a9)

## Automate Your Robot 
Load Controller Launch file to automate the robot
```bash
ros2 launch solar_robot_controller_pkg controller_estimator.launch.py
```
Let’s see what topics are active. Open a new terminal, and type:
```bash
ros2 topic list -t
```
## Solar Autobot in the World Moving in search of light source
![Robot](https://github.com/Tarunesh38/Solar_Autobot_Batch-B_17/assets/119646778/190ca019-1d24-43fe-8282-fd86489d9be3)
![Robot2](https://github.com/Tarunesh38/Solar_Autobot_Batch-B_17/assets/119646778/46ab11ee-8275-406a-b443-2572d3b76d42)

## Manually move the robot
Open a new terminal:
```bash
ros2 launch solar_robot_spawner_pkg gazebo_world.launch.py
```
Again, Open a new Terminal and Type:
```bash
sudo apt install ros-humble-turtlebot3*
```
```bash
ros2 run turtlebot3_teleop teleop_keyboard --ros-args --remap /cmd_vel:=/demo/cmd_vel
```

## Simulation
https://github.com/Tarunesh38/Solar_Autobot_Batch-B_17/assets/119646778/e0bdb9b3-3da0-4e1c-961a-e032e239929c


## Spawing the camera in Rviz
Open a new terminal
```bash
ros2 launch solar_robot_spawner_pkg gazebo_world.launch.py
```
Again, Open a new Terminal and Type:
```bash
rviz2
```
In Rviz,
--> Set the Fixed Frame to Odom
--> Click add and select Image
--> Under Images tab, select Topic as image.raw
--> Now image tab opens and you can see what the robot captures
![Robot5](https://github.com/Tarunesh38/Solar_Autobot_Batch-B_17/assets/119646778/696c20b8-3ce5-4290-b4be-19158cb71e47)

## Hardware
![Hardware](https://github.com/Tarunesh38/Solar_Autobot_Batch-B_17/assets/119646778/b4960dce-6e97-4154-9555-eb89739deaf9)

## References
```bash
https://automaticaddison.com/how-to-simulate-a-robot-using-gazebo-and-ros-2/
https://automaticaddison.com/how-to-create-an-object-following-robot-ros-2-navigation/
https://classic.gazebosim.org/tutorials?tut=led_plugin
http://models.gazebosim.org/
```

