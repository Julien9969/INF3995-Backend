import datetime
import asyncio
from .identify_client import IdentifyClientAsync
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from pydantic import BaseModel


connected_robots = set()
i = 1

class IdentifyBase:
    # def __init__(self):
    #     pass

    @staticmethod
    async def launch_client(robot_id: int = 1) -> str:
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
    

    @staticmethod
    async def list_connected_robot() -> list[int]:
        global connected_robots, i
        connected_robots = set()
        rclpy.init()

        
        for i in range(1, 5):
            node = rclpy.create_node('robot_connector_node')
            odom_topic = f'/robot{i}/odom'
            subscriber = node.create_subscription(Odometry, odom_topic, IdentifyBase.odom_callback, 10)
            node.get_logger().info(f"Connected robots: {connected_robots}")
            rclpy.spin_once(node, timeout_sec=1)
            node.destroy_node()
            
        await asyncio.sleep(1)  # Wait for some time to receive odom data
        rclpy.shutdown()


        return list(connected_robots)

    @staticmethod
    def odom_callback(msg):
        global connected_robots, i
        connected_robots.add(i)
        # Here you can implement your logic to process the received odom data
        # For example, print the received data along with the robot's ID
        print(f"Received odom data from robot {i} :")
    
class IdentifyResponse(BaseModel):
    data: str




