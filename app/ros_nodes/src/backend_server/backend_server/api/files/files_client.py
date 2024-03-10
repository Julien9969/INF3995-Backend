import asyncio
import sys
from interfaces.srv import FilesServer
import rclpy
from rclpy.node import Node

class FilesClientAsync(Node):
    """
    This class is used to call the ROS service 'files' from the backend.
    """
    def __init__(self, robot_id: int = 1):
        super().__init__('files_client_async')
        ros_route = f"robot{robot_id}/files"
        self.cli = self.create_client(FilesServer, ros_route)

        if self.cli.wait_for_service(timeout_sec=5.0):
            self.req = FilesServer.Request()
        else:    
            self.get_logger().info(f'service not available (robot id {robot_id}).')

    async def send_request(self, command: str, content: str = "None"):
        self.req.command = command
        self.req.content = content
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()