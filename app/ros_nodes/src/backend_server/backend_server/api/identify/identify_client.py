import sys
from interfaces.srv import Identify
import rclpy
from rclpy.node import Node

class IdentifyClientAsync(Node):
    """
    This class is used to call the ROS service 'identify' from the backend.
    """
    def __init__(self, robot_id: int = 1):
        super().__init__('identify_client_async')
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
