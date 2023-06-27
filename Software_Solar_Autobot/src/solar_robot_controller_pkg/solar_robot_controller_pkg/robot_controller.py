# Author: Your Name
# Date: June 27, 2023
# ROS Version: ROS 2 Foxy Fitzroy

############## IMPORT LIBRARIES #################
# Python math library
import math

# Random number generator
import random

# ROS client library for Python
import rclpy

# Enables pauses in the execution of code
from time import sleep

# Used to create nodes
from rclpy.node import Node

# Enables the use of the string message type
from std_msgs.msg import String

# Twist is linear and angular velocity
from geometry_msgs.msg import Twist

# Handles LaserScan messages to sense distance to obstacles (i.e. walls)
from sensor_msgs.msg import LaserScan

# Handle Pose messages
from geometry_msgs.msg import Pose

# Handle float64 arrays
from std_msgs.msg import Float64MultiArray

# Handles quality of service for LaserScan data
from rclpy.qos import qos_profile_sensor_data

# Scientific computing library
import numpy as np


class Controller(Node):
    """
    Create a Controller class, which is a subclass of the Node
    class for ROS2.
    """

    def __init__(self):
        """
        Class constructor to set up the node
        """
        ##################### ROS SETUP ####################################################
        # Initiate the Node class's constructor and give it a name
        super().__init__('Controller')

        # Create a publisher
        # This node publishes the desired linear and angular velocity of the robot
        # (in the robot chassis coordinate frame) to the /demo/cmd_vel topic.
        self.publisher_ = self.create_publisher(
            Twist,
            '/demo/cmd_vel',
            10)

        ################### ROBOT CONTROL PARAMETERS ##################
        # Maximum forward speed of the robot in meters per second
        # Any faster than this and the robot risks falling over.
        self.forward_speed = 0.2

        # Start moving the robot randomly
        self.move_randomly()

    def move_randomly(self):
        """
        Move the robot randomly in the world
        """
        # Create a geometry_msgs/Twist message
        msg = Twist()
        msg.linear.x = self.forward_speed
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0

        # Randomly decide whether to rotate or move straight
        if random.random() < 0.2:
            # Rotate randomly between -pi/4 and pi/4 radians
            msg.angular.z = random.uniform(-math.pi / 4, math.pi / 4)
        else:
            # Move straight
            msg.angular.z = 0.0

        # Publish the velocity command to the robot
        self.publisher_.publish(msg)

    def stop_robot(self):
        """
        Stop the robot's movement
        """
        # Create a geometry_msgs/Twist message
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0

        # Publish the velocity command to the robot
        self.publisher_.publish(msg)


def main(args=None):

    # Initialize rclpy library
    rclpy.init(args=args)

    # Create the node
    controller = Controller()

    while True:
        # Move straight for 15 seconds
        sleep(15)

        # Stop the robot's movement
        controller.stop_robot()

        # Rotate randomly to the left or right
        if random.random() < 0.5:
            # Rotate to the left
            controller.move_randomly()
        else:
            # Rotate to the right
            controller.move_randomly()

        # Keep rotating for 5 seconds
        sleep(5)

        # Stop the robot's movement
        controller.stop_robot()

        # Move straight again
        controller.move_randomly()

    # Spin the node so the callback function is called
    # Pull messages from any topics this node is subscribed to
    # Publish any pending messages to the topics
    rclpy.spin(controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    controller.destroy_node()

    # Shutdown the ROS client library for Python
    rclpy.shutdown()


if __name__ == '__main__':
    main()

