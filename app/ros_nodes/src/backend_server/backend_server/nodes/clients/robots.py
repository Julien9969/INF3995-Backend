import asyncio
import time

import rclpy
import logging

from backend_server.nodes.clients.identify import IdentifyClientAsync
from fastapi.concurrency import run_in_threadpool
from geometry_msgs.msg import Twist
from interfaces.srv import Identify
from nav_msgs.msg import Odometry
from rclpy.node import Node

from backend_server.logic.robots import RobotsData


class RobotNode(Node):

    def __init__(self, robot_id: int = 1):
        super().__init__('identify_client_async')
        self.future = None
        ros_route = f"robot{robot_id}/identify"
        self.cli = self.create_client(Identify, ros_route)

        if self.cli.wait_for_service(timeout_sec=5.0):
            self.req = Identify.Request()
        else:
            self.get_logger().info(f'service not available (robot id {robot_id}), waiting again...')

    async def send_request(self, a):
        self.req.a = a
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)

        return self.future.result()

    async def launch_client(self, robot_id: int = 1) -> str:

        identify_client = IdentifyClientAsync(robot_id)

        if not hasattr(identify_client, 'req'):
            identify_client.destroy_node()

            return None

        response = await identify_client.send_request(4)
        self.get_logger().info(
            'Result of identify: for %d * 2 = %d' %
            (4, response.b))

        identify_client.destroy_node()

        return 'Result of identify: for %d * 2 = %d' % (4, response.b)

    async def get_connected_robot(self) -> list[int]:
        connected_robots = set()
        for i in range(1, 3):
            node = rclpy.create_node('robot_connector_node')
            odom_topic = f'/robot{i}/odom'
            subscriber = node.create_subscription(Odometry, odom_topic, self.odom_callback, 10)

            self.get_logger().info(f"Connected robots: {connected_robots}")

            await run_in_threadpool(lambda: rclpy.spin_once(node, timeout_sec=5))
            await asyncio.sleep(1)  # Wait for some time to receive odom data
            node.destroy_node()

        return list(connected_robots)

    def odom_callback(self, msg):
        robots = RobotsData()
        robots.add_robot(msg)
        logging.debug(f"Received odom data from robot {i} :")

    def send_cmd_vel(self, x: float, z: float):
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
