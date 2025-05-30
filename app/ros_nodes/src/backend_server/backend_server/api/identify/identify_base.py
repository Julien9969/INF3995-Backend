import asyncio
import logging
import re

import rclpy
from backend_server.models.robots import RobotsData
from fastapi.concurrency import run_in_threadpool
from nav_msgs.msg import Odometry
from pydantic import BaseModel

MAX_ROBOT_INDEX = 3
logging.basicConfig(level=logging.DEBUG)

from .identify_client import IdentifyClientAsync

connected_robots = set()
i = 1


class IdentifyBase:
    connected_robots = set()

    @staticmethod
    async def launch_client(robot_id: int = 1) -> str:

        identify_client = IdentifyClientAsync(robot_id)

        if not hasattr(identify_client, 'req'):
            identify_client.destroy_node()

            return None

        response = await identify_client.send_request(4)
        logging.info('Result of identify: for %d * 2 = %d' % (4, response.b))

        identify_client.destroy_node()

        return 'Result of identify: for %d * 2 = %d' % (4, response.b)

    @staticmethod
    async def list_connected_robot() -> list[int]:
        for i in range(1, MAX_ROBOT_INDEX):
            node = rclpy.create_node('robot_connector_node')
            odom_topic = f'/robot{i}/odom'
            logging.info(f"trying  {odom_topic}")
            subscriber = node.create_subscription(Odometry, odom_topic, IdentifyBase.odom_callback, 10)

            logging.info(f"Connected  to robots {IdentifyBase.connected_robots}")

            try:
                await run_in_threadpool(lambda: rclpy.spin_once(node, timeout_sec=5))
            except Exception as e:
                logging.info(f"Identify connected already executing {odom_topic} will wait his response")
                await asyncio.sleep(1)
                node.destroy_node()
                return str(IdentifyBase.connected_robots)
            
            await asyncio.sleep(1)  # Wait for some time to receive odom data
            node.destroy_node()

        RobotsData().reset_robots(IdentifyBase.connected_robots)

        return str([robot.id for robot in RobotsData().robots])

    def get_robot_id(self, name: str):

        pattern = r"robot(\d+)"
        match = re.search(pattern, name)
        if match:
            robot_id = match.group(1)
            return int(robot_id)
        else:
            return 0

    @staticmethod
    def odom_callback(msg):
        robot_id = IdentifyBase().get_robot_id(msg.header.frame_id)
        IdentifyBase.connected_robots.add(robot_id)
        logging.info(f"Received odom data from robot {robot_id} :")


class IdentifyResponse(BaseModel):
    data: str
