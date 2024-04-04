import asyncio
import datetime
import time

import rclpy
from geometry_msgs.msg import Twist
from pydantic import BaseModel


class PingBase:

    @staticmethod
    def ping():
        return "pong"

    @staticmethod
    def send_cmd_vel(x: float, z: float, robot: bool):
        # TODO: could be moved into the node folder
        if not rclpy.ok():
            rclpy.init()

        node = rclpy.create_node('twist_publisher')
        publisher = node.create_publisher(Twist, 'robot2/cmd_vel', 10)

        msg = Twist()
        msg.linear.x = x
        msg.angular.z = z

        for i in range(50):
            publisher.publish(msg)
            node.get_logger().info('Publishing: {}'.format(msg))
            time.sleep(0.3)

        node.destroy_node()

        return asyncio.sleep(0)


class PingResponse(BaseModel):
    data: str
