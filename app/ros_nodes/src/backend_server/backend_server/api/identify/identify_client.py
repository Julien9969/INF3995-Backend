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
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = Identify.Request()

    def send_request(self, a):
        self.req.a = a
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


# def main():
#     rclpy.init()

#     identify_client = IdentifyClientAsync()
#     response = identify_client.send_request(4)
#     identify_client.get_logger().info(
#         'Result of add_two_ints: for %d * 2 = %d' %
#         (4, response.b))

#     identify_client.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()