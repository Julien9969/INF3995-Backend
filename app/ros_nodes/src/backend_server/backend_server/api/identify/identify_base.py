import datetime
import asyncio
from .identify_client import IdentifyClientAsync
import rclpy
from geometry_msgs.msg import Twist
from pydantic import BaseModel


class IdentifyBase:
    def __init__(self):
        pass

    @staticmethod
    async def send_cmd_vel():
        # Initialize ROS node
        rclpy.init()
        node = rclpy.create_node('cmd_vel_publisher')

        # Create a publisher for the cmd_vel topic
        publisher = node.create_publisher(Twist, '/limo/cmd_vel', 10)

        # Create a Twist message to send velocity commands
        twist_msg = Twist()
        twist_msg.linear.x = -0.1  # Example linear velocity
        twist_msg.angular.z = -2.0  # Example angular velocity

        # Publish the Twist message
        publisher.publish(twist_msg)

        node.destroy_node()
        rclpy.shutdown()
    
    @staticmethod
    async def launch_client(robot_id: int = 1):
        rclpy.init()
        identify_client = IdentifyClientAsync(robot_id)

        # if not hasattr(identify_client, 'req'):
        if not hasattr(identify_client, 'req'):
            identify_client.destroy_node()
            rclpy.shutdown()
            return None

        response = identify_client.send_request(4)
        identify_client.get_logger().info(
            'Result of identify: for %d * 2 = %d' %
            (4, response.b))

        identify_client.destroy_node()
        rclpy.shutdown()
        return 'Result of identify: for %d * 2 = %d' %(4, response.b)
    
class IdentifyResponse(BaseModel):
    data: str




