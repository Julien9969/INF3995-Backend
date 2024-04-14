import logging
from backend_server.api.identify.identify_base import IdentifyBase
import rclpy
from interfaces.srv import MissionSwitch
from rclpy.node import Node

logging.basicConfig(level=logging.INFO)

class MissionNode(Node):
    """
    This class is used to call the ROS service 'identify' from the backend.
    """

    def __init__(self):
        super().__init__('mission_switch_client_async')
        self._futures:dict = {}
        self._clients:list = []
        self._client_ids:dict = {}

        try:
            for id in IdentifyBase.connected_robots:
                ros_route = f"/robot{id}/mission_switch"
                client = self.create_client(MissionSwitch, ros_route)
                if client.wait_for_service(timeout_sec=5.0):
                    self.req = MissionSwitch.Request()
                self._clients.append(client)
                self._client_ids[id] = len(self._clients) - 1
        except Exception as e:
            logging.error(f"Error creating ROS clients: {e}")
            raise
    
    def send_request(self, cmd: str):
        self.req.command = cmd  
        for id in self._client_ids:
            self._futures[id] = self._clients[self._client_ids[id]].call_async(self.req)
            rclpy.spin_until_future_complete(self, self._futures[id])
        return {id: future.result() for id, future in self._futures.items()}

    def start_mission(self):
        if not hasattr(self, 'req'):
            self.destroy_node()
            return None

        responses = self.send_request('start')
        environments = {id: response.environment for id, response in responses.items()}
        answers = {id: response.answer for id, response in responses.items()}
            
        if len(set(environments.values())) > 1:
            logging.info("Error: different environments returned in responses")
            self.destroy_node()
            return answers, None

        self.destroy_node()        
        return answers, next(iter(environments.values()))

    def stop_mission(self):
        if not hasattr(self, 'req'):
            self.destroy_node()
            return None

        responses = self.send_request('stop')
        self.destroy_node()
        return responses