import logging
import rclpy
from interfaces.srv import MissionSwitch
from rclpy.node import Node


class MissionNode(Node):
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
            if self.cli1.wait_for_service(timeout_sec=2.0):  # TODO: could this be in parallel
                if self.cli2.wait_for_service(timeout_sec=2.0):
                    self.req = MissionSwitch.Request()
        except Exception as e:
            logging.error(f"Error creating ROS clients: {e}")
            raise

    def send_request(self, cmd: str):
        self.req.command = cmd  # TODO: what if more that tree robots
        self.future1 = self.cli1.call_async(self.req)
        self.future2 = self.cli2.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future1)
        rclpy.spin_until_future_complete(self, self.future2)
        return self.future1.result(), self.future2.result()

    def start_mission(self):
        if not rclpy.ok():
            rclpy.init()

        if not hasattr(self, 'req'):
            self.destroy_node()
            return None

        response1, response2 = self.send_request('start')
        result = f"{response1}, {response2}"  # TODO: what if more that tree robots

        self.destroy_node()
        return result

    def stop_mission(self):
        if not rclpy.ok():
            rclpy.init()

        if not hasattr(self, 'req'):
            self.destroy_node()
            return None

        response1, response2 = self.send_request('stop')
        result = f"Robots response to stop: {response1}, {response2}"

        self.destroy_node()
        return result
