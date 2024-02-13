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
    def send_cmd_vel(x: float, z: float, robot: bool):
        rclpy.init()
        node = rclpy.create_node('cmd_vel_publisher')
        print("Sending cmd_vel", x, z, robot)

        publisher = node.create_publisher(Twist, '/cmd_vel', 10)

        twist_msg = Twist()
        twist_msg.linear.x = x 
        twist_msg.angular.z = z  

        publisher.publish(twist_msg)

        node.destroy_node()
        rclpy.shutdown()
    
class PingResponse(BaseModel):
    data: str




