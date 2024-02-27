from .navigate_client import NavigateClientAsync
import rclpy
from pydantic import BaseModel

class NavigateBase:
    def __init__(self):
        pass

    @staticmethod
    async def send_cmd_vel():
        # Initialize ROS node
        rclpy.init()
        node = rclpy.create_node('cmd_vel_publisher')

        # Create a publisher for the cmd_vel topic
        publisher = node.create_publisher()

        # Publish the Twist message
        publisher.publish(twist_msg)

        node.destroy_node()
        rclpy.shutdown()
    
    @staticmethod
    async def launch_client(robot_id: int = 1):
        rclpy.init()
        navigate_client = NavigateClientAsync(robot_id)

        if not hasattr(navigate_client, 'req'):
            navigate_client.destroy_node()
            rclpy.shutdown()
            return None

        response = navigate_client.send_request(4)
        navigate_client.get_logger().info(
            'Result of navigate %d'% (response.b))

        navigate_client.destroy_node()
        rclpy.shutdown()
        return 'Result of navigate %d'% (response.b)
    
class NavigateResponse(BaseModel):
    data: str




