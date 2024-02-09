#!/usr/bin/env python3

import asyncio
import rclpy
from geometry_msgs.msg import Twist

def send_cmd_vel():
    # Initialize ROS node
    rclpy.init()
    node = rclpy.create_node('cmd_vel_publisher')

    # Create a publisher for the cmd_vel topic
    publisher = node.create_publisher(Twist, 'limo/cmd_vel', 10)

    # Create a Twist message to send velocity commands
    twist_msg = Twist()
    twist_msg.linear.x = 0.0  # Example linear velocity
    twist_msg.angular.z = 3.0  # Example angular velocity

    # Publish the Twist message
    publisher.publish(twist_msg)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    send_cmd_vel()
