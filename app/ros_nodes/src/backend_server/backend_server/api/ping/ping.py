import datetime
import asyncio
import rclpy
from geometry_msgs.msg import Twist
from pydantic import BaseModel

class PingBase:
    def __init__(self):
        pass

    @staticmethod
    def ping():
        return f"{datetime.datetime.now().strftime('%d/%m/%Y, %H:%M:%S')} - pong!"
    
    @staticmethod
    def send_cmd_vel():
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
    
class PingResponse(BaseModel):
    data: str




