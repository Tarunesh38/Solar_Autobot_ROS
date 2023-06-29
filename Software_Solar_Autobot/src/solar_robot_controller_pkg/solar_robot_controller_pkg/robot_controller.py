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

# Import OpenCV
import cv2
import numpy as np

# Import additional ROS message types
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

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

        ##################### LIDAR SETUP #############################
        # Create a subscriber
        # This node subscribes to the LaserScan data from the /demo/laser_scan topic.
        self.laser_subscriber_ = self.create_subscription(
            LaserScan,
            '/demo/laser_scan',
            self.laser_callback,
            qos_profile=qos_profile_sensor_data)

        ##################### CAMERA SETUP ############################
        # Create a subscriber
        # This node subscribes to the Image data from the /demo/camera/image_raw topic.
        self.image_subscriber_ = self.create_subscription(
            Image,
            '/demo/camera/image_raw',
            self.image_callback,
            qos_profile=10)

        # Create an instance of CvBridge
        self.cv_bridge = CvBridge()
        self.target_color = [255, 255, 255]

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

    def laser_callback(self, msg):
        """
        Callback function for the LaserScan data
        """
        # Process the LaserScan data to avoid obstacles
        # ... Add your collision avoidance logic here ...

        # Stop the robot if an obstacle is too close
        if min(msg.ranges) < 0.5:
            self.stop_robot()

    def image_callback(self, msg):
        """
        Callback function for the Image data
        """
        # Convert the Image message to OpenCV format
        cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        # Find the centroid of the target color in the image
        centroid = self.find_color_centroid(cv_image)

        # Adjust the robot's velocity based on the centroid position
        self.adjust_velocity(centroid)

    def find_color_centroid(self, image):
        # Convert the image to the HSV color space for easier color detection
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Define the range of the target color in HSV format
        lower_color = np.array([0, 0, 200])
        upper_color = np.array([180, 20, 255])

        # Threshold the image to only keep pixels within the color range
        mask = cv2.inRange(hsv_image, lower_color, upper_color)

        # Find contours in the thresholded image
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Find the centroid of the largest contour
        if len(contours) > 0:
            largest_contour = max(contours, key=cv2.contourArea)
            M = cv2.moments(largest_contour)
            if M['m00'] > 0:
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                centroid = (cx, cy)
                return centroid

        return None
        
    def main(self):
        """
        Main function to control the robot
        """
        while True:
            # Move straight for 15 seconds
            sleep(5)

            # Stop the robot's movement
            self.stop_robot()

            # Rotate randomly to the left or right
            if random.random() < 0.5:
                # Rotate to the left
                self.move_randomly()
            else:
                # Rotate to the right
                self.move_randomly()

            # Keep rotating for 2 seconds
            sleep(2)

            # Stop the robot's movement
            self.stop_robot()

            # Move straight again
            self.move_randomly()

def main(args=None):
    # Initialize rclpy library
    rclpy.init(args=args)

    # Create the node
    controller = Controller()

    try:
        # Run the main loop
        controller.main()
    except KeyboardInterrupt:
        # Handle Ctrl+C interruption
        pass

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    controller.destroy_node()

    # Shutdown the ROS client library for Python
    rclpy.shutdown()


if __name__ == '__main__':
    main()
