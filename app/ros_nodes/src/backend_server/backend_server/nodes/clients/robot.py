# TODO: retrieve battery level of the robot
# TODO: retrieve health check of the the robot (state)
# TODO: retrieve the run distance of the robot
# TODO: retrieve the current position of the robot
import rclpy
from interfaces.srv import Identify
from pydantic import BaseModel
from rclpy.node import Node
from backend_server.constants import RCL_TIMEOUT


class IdentifyBase:

    @staticmethod
    async def launch_client(robot_id: int = 1):
        if not rclpy.ok():
            rclpy.init()
        identify_client = IdentifyClientAsync(robot_id)

        if not hasattr(identify_client, 'req'):
            identify_client.destroy_node()
            return None

        response = await identify_client.send_request(4)
        identify_client.get_logger().info(
            'Result of identify: for %d * 2 = %d' %
            (4, response.b))

        identify_client.destroy_node()
        return 'Result of identify: for %d * 2 = %d' % (4, response.b)


class IdentifyResponse(BaseModel):
    data: str


class IdentifyClientAsync(Node):
    """
    This class is used to call the ROS service 'identify' from the backend.
    """

    def __init__(self, robot_id: int = 1):
        super().__init__('identify_client_async')
        ros_route = f"robot{robot_id}/identify"
        self.cli = self.create_client(Identify, ros_route)

        if self.cli.wait_for_service(timeout_sec=RCL_TIMEOUT):
            self.req = Identify.Request()
        else:
            self.get_logger().info(f'service not available (robot id {robot_id}), waiting again...')

    async def send_request(self, a):
        self.req.a = a
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)

        return self.future.result()
