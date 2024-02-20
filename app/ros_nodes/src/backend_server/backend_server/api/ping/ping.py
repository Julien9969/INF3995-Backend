import datetime, time
import asyncio
import rclpy
from geometry_msgs.msg import Twist
from pydantic import BaseModel

class PingBase:
    # def __init__(self):
    #     pass

    @staticmethod
    def ping():
        return f"{datetime.datetime.now().strftime('%d/%m/%Y, %H:%M:%S')} - pong!"
    
    @staticmethod
    def send_cmd_vel(x: float, z: float, robot: bool):
        rclpy.init()

        node = rclpy.create_node('twist_publisher')
        # rate = node.create_timer(1, )
        publisher = node.create_publisher(Twist, '/cmd_vel', 10)

        msg = Twist()
        msg.linear.x = x
        msg.angular.z = z

        # rate = node.create_rate(1)  # 1 Hz
        for i in range(50):
            publisher.publish(msg)
            node.get_logger().info('Publishing: {}'.format(msg))
            time.sleep(0.3)
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




