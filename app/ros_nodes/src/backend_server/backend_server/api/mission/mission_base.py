
from interfaces.srv import MissionSwitch
import rclpy
from rclpy.node import Node


# TODO Logigique de base modifiable au besoin 
class MissionBase:
    """Singleton mission base class."""
    mission = None

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

        # if not hasattr(identify_client, 'req'):
        if not hasattr(mission_client, 'req'):
            mission_client.destroy_node()
            rclpy.shutdown()
            return None

        response1, response2 = mission_client.send_request('start')
        
        mission_client.get_logger().info(f"{response1}, {response2}")

        mission_client.destroy_node()
        rclpy.shutdown()

        return "Mission started !"

    @staticmethod
    def stop_mission():
        """Stop mission."""
        # mission = MissionBase.get_mission()
        # mission.stop()
        rclpy.init()
        mission_client = Mission()

        if not hasattr(mission_client, 'req'):
            mission_client.destroy_node()
            rclpy.shutdown()
            return None
        
        response1, response2 = mission_client.send_request('stop')

        mission_client.get_logger().info(f"{response1}, {response2}")

        mission_client.destroy_node()
        rclpy.shutdown()

        return "Mission stopped"


class Mission(Node):
    """
    This class is used to call the ROS service 'identify' from the backend.
    """
    def __init__(self):
        super().__init__('identify_client_async')
        ros_route = f"robot{1}/mission_switch"
        self.cli1 = self.create_client(MissionSwitch, ros_route)

        ros_route = f"robot{2}/mission_switch"
        self.cli2 = self.create_client(MissionSwitch, ros_route)

        if self.cli1.wait_for_service(timeout_sec=5.0):
            if self.cli2.wait_for_service(timeout_sec=5.0):
                self.req = MissionSwitch.Request()
        else:
            self.get_logger().info(f'Mission service not available, waiting again...')

    def send_request(self, cmd: str):
        self.req.command = cmd
        self.future1 = self.cli1.call_async(self.req)
        self.future2 = self.cli2.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future1)
        rclpy.spin_until_future_complete(self, self.future2)
        return self.future1.result(), self.future2.result()


