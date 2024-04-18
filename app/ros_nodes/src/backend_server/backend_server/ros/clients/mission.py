import logging

import rclpy
from backend_server.api.identify.identify_base import IdentifyBase
from interfaces.srv import MissionSwitch
from rclpy.node import Node

logging.basicConfig(level=logging.INFO)


class MissionNode(Node):
    """
    This class is used to call the ROS service 'identify' from the backend.
    """

    def __init__(self):
        super().__init__('mission_switch_client_async')
        self._futures: dict = {}
        self._clients: list = []
        self._client_ids: dict = {}

        try:
            for robot_id in IdentifyBase.connected_robots:
                ros_route = f"/robot{robot_id}/mission_switch"
                client = self.create_client(MissionSwitch, ros_route)
                if client.wait_for_service(timeout_sec=5.0):
                    self.req = MissionSwitch.Request()
                self._clients.append(client)
                self._client_ids[robot_id] = len(self._clients) - 1
        except Exception as e:
            logging.error(f"Error creating ROS clients: {e}")
            raise

    def _check_req_exists(self):
        if not hasattr(self, 'req'):
            self.destroy_node()
            return False
        return True

    def send_request(self, cmd: str):
        if not self._check_req_exists():
            return None

        self.req.command = cmd
        for robot_id in self._client_ids:
            self._futures[robot_id] = self._clients[self._client_ids[robot_id]].call_async(self.req)
            rclpy.spin_until_future_complete(self, self._futures[robot_id])
        return {robot_id: future.result() for robot_id, future in self._futures.items()}

    def start_mission(self):
        if not self._check_req_exists():
            return None

        responses = self.send_request('start')
        environments = {response.environment for response in responses.values()}
        answers = {id: response.answer for id, response in responses.items()}

        if len(environments) > 1:
            logging.info("Error: different environments returned in responses")
            self.destroy_node()
            return answers, None

        self.destroy_node()
        return answers, next(iter(environments))

    def stop_mission(self):
        if not self._check_req_exists():
            return None

        responses = self.send_request('stop')
        self.destroy_node()
        return responses

    def head_back_base(self, robot_id:int):
        if not self._check_req_exists():
            return None

        if robot_id not in self._client_ids:
            logging.error(f"Robot ID {robot_id} not found")
            return None

        self.req.command = 'home'
        logging.info(f"Sending request to robot {robot_id} to head back to base")
        self._futures[robot_id] = self._clients[self._client_ids[robot_id]].call_async(self.req)
        rclpy.spin_until_future_complete(self, self._futures[robot_id])
        response = self._futures[robot_id].result()
        self.destroy_node()
        return {robot_id: response}