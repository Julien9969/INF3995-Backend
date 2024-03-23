import os

from interfaces.srv import MissionSwitch

import rclpy
from rclpy.node import Node

import time

ROBOT_COUNT = 2  # Store the number of robots as a constant

SIMULATION = os.environ.get('ROS_DOMAIN_ID') == '0'

from enum import Enum


class MissionState(str, Enum):
    ONGOING = "ongoing"
    ENDED = "ended"
    NOT_STARTED = "not-started"


# Source: https://stackoverflow.com/questions/6760685/what-is-the-best-way-of-implementing-singleton-in-python
class Singleton(type):
    _instances = {}

    def __call__(cls, *args, **kwargs):
        if cls not in cls._instances:
            cls._instances[cls] = super(Singleton, cls).__call__(*args, **kwargs)
        return cls._instances[cls]


def start_mission():
    if(not rclpy.ok()):
        rclpy.init()
    mission_client = Mission()
    MissionData().start_timestamp = int(time.time())
    MissionData().stop_timestamp = 0
    MissionData().state = MissionState.ONGOING

    if not hasattr(mission_client, 'req'):
        mission_client.destroy_node()
        return None

    response1, response2 = mission_client.send_request('start')
    result = f"{response1}, {response2}"
    # mission_client.get_logger().info(result)

    mission_client.destroy_node()
    return result


def stop_mission():
    if(not rclpy.ok()):
        rclpy.init()
    mission_client = Mission()
    # MissionData().stop_timestamp = int(time.time())

    MissionData().stop_timestamp = int(time.time())
    MissionData().state = MissionState.ENDED
    
    if not hasattr(mission_client, 'req'):
        mission_client.destroy_node()
        return None

    response1, response2 = mission_client.send_request('stop')
    result = f"Robots response to stop: {response1}, {response2}"
    # mission_client.get_logger().info(result)

    mission_client.destroy_node()
    return result


class Mission(Node):
    """
    This class is used to call the ROS service 'identify' from the backend.
    """

    def __init__(self):
        super().__init__('identify_client_async')
        self.future2 = None
        self.future1 = None

        try:
            ros_route = f"robot{1}/mission_switch"
            self.cli1 = self.create_client(MissionSwitch, ros_route)

            ros_route = f"robot{2}/mission_switch"
            self.cli2 = self.create_client(MissionSwitch, ros_route)
            if self.cli1.wait_for_service(timeout_sec=5.0):  # Active Waiting
                if self.cli2.wait_for_service(timeout_sec=5.0):
                    self.req = MissionSwitch.Request()
        except Exception as e:
            self.get_logger().error(f"Error creating ROS clients: {e}")
            raise

    def send_request(self, cmd: str):
        self.req.command = cmd
        self.future1 = self.cli1.call_async(self.req)
        self.future2 = self.cli2.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future1)
        rclpy.spin_until_future_complete(self, self.future2)
        return self.future1.result(), self.future2.result()


class MissionData(metaclass=Singleton):

    def __init__(self):
        self.start_timestamp: int = 0
        self.stop_timestamp: int = 0
        self.state = MissionState.NOT_STARTED

    def get_mission_state(self):
        return self.state.value

    def get_mission_duration(self):
        if self.stop_timestamp != 0:
            return self.stop_timestamp - self.start_timestamp
        else:
            return int(time.time()) - self.start_timestamp

    def get_battery(self):
        return [50, 50]  # TODO

    def get_robot_status(self):
        return [0.2, 0.2]  # TODO
