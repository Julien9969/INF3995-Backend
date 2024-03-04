import datetime
import asyncio
from .identify_client import IdentifyClientAsync
import rclpy
from geometry_msgs.msg import Twist
from pydantic import BaseModel

class IdentifyBase:
    # def __init__(self):
    #     pass

    @staticmethod
    async def launch_client(robot_id: int = 1):
        rclpy.init()
        identify_client = IdentifyClientAsync(robot_id)

        if not hasattr(identify_client, 'req'):
            identify_client.destroy_node()
            rclpy.shutdown()
            return None

        response = await identify_client.send_request(4)
        identify_client.get_logger().info(
            'Result of identify: for %d * 2 = %d' %
            (4, response.b))

        identify_client.destroy_node()
        rclpy.shutdown()
        return 'Result of identify: for %d * 2 = %d' % (4, response.b)
    
class IdentifyResponse(BaseModel):
    data: str




