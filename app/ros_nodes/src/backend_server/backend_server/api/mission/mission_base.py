import os

from interfaces.srv import MissionSwitch
import rclpy
from rclpy.node import Node
import time


class MissionBase:
    """
    Singleton for mission, which also includes robot information
    """
    mission = None

    # TODO: Add timestamp for start/stop to compute duration
    # TODO: Create a getter for mission status (to be )
    # TODO: Create a getter for battery level
    # TODO: Add env variable to know if running in dev

    @staticmethod
    def get_mission():
        """Get mission object."""
        if MissionBase.mission is None:
            MissionBase.mission = Mission()
        return MissionBase.mission

    @staticmethod
    def start_mission():
        """Start mission."""
        rclpy.init()
        mission_client = Mission()
        mission_client.start_timestamp = int(time.time())
        # if not hasattr(identify_client, 'req'):
        if not hasattr(mission_client, 'req'):
            mission_client.destroy_node()
            rclpy.shutdown()
            return None

        response1, response2 = mission_client.send_request('start')
        result = f"{response1}, {response2}"
        mission_client.get_logger().info(result)

        mission_client.destroy_node()
        rclpy.shutdown()
        # TODO: add error handling
        return result

    @staticmethod
    def stop_mission():
        """Stop mission."""
        rclpy.init()
        mission_client = Mission()
        mission_client.stop_timestamp = int(time.time())
        if not hasattr(mission_client, 'req'):
            mission_client.destroy_node()
            rclpy.shutdown()
            return None

        response1, response2 = mission_client.send_request('stop')
        result = f"Robots response to stop: {response1}, {response2}"
        mission_client.get_logger().info(result)

        mission_client.destroy_node()
        rclpy.shutdown()

        return result

    @staticmethod
    def is_mission_ongoing():
        return MissionBase.mission is not None

    @staticmethod
    def get_mission_duration():
        """
        To be used when persisting data into the database
        """
        if Mission.stop_timestamp != 0:
            return Mission.stop_timestamp - Mission.start_timestamp
        else:
            return 0


class Mission(Node):
    """
    This class is used to call the ROS service 'identify' from the backend.
    """
    start_timestamp: int = 0
    stop_timestamp: int = 0
    mission_name: str = "Mission"

    def __init__(self):
        super().__init__('identify_client_async')
        self.future2 = None
        self.future1 = None
        ros_route = f"robot{1}/mission_switch"
        self.cli1 = self.create_client(MissionSwitch, ros_route)

        ros_route = f"robot{2}/mission_switch"
        self.cli2 = self.create_client(MissionSwitch, ros_route)
        debug = os.getenv("DEBUG") if os.getenv("DEBUG") is not None else False
        if self.cli1.wait_for_service(timeout_sec=5.0) and not debug:  # Active Waiting
            if self.cli2.wait_for_service(timeout_sec=5.0):
                self.req = MissionSwitch.Request()

    def send_request(self, cmd: str):
        self.req.command = cmd
        self.future1 = self.cli1.call_async(self.req)
        self.future2 = self.cli2.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future1)
        rclpy.spin_until_future_complete(self, self.future2)
        return self.future1.result(), self.future2.result()
