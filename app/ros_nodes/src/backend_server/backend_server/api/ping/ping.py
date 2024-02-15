import datetime, time
import asyncio
import rclpy
from geometry_msgs.msg import Twist
from pydantic import BaseModel
from trajectory_msgs.msg import JointTrajectoryPoint

class PingBase:
    def __init__(self):
        pass

    @staticmethod
    def ping():
        return f"{datetime.datetime.now().strftime('%d/%m/%Y, %H:%M:%S')} - pong!"
    
    @staticmethod
    def send_cmd_vel(x: float, z: float, robot: bool):
        rclpy.init()

        node = rclpy.create_node('twist_publisher')
        rate = node.create_rate(2)
        publisher = node.create_publisher(Twist, '/cmd_vel', 10)

        msg = Twist()
        msg.linear.x = x
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = z

        print('oiehc')
        # rate = node.create_rate(1)  # 1 Hz
        for i in range(5):
            publisher.publish(msg)
            node.get_logger().info('Publishing: {}'.format(msg))
            rate.sleep()
            # rate.sleep()

        node.destroy_node()
        rclpy.shutdown()

        return asyncio.sleep(0)

        # # Spin briefly to allow message to be published
        # rclpy.spin_once(node, timeout_sec=2.5)


        # from time import sleep
        # sleep(2)
        # node.destroy_node()
        # rclpy.shutdown()
    
class PingResponse(BaseModel):
    data: str




