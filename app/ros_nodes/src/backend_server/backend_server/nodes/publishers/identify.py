from geometry_msgs.msg import Twist
import rclpy
import time
import asyncio


def send_cmd_vel(x: float, z: float, robot: bool):
    if not rclpy.ok():
        rclpy.init()

    node = rclpy.create_node('twist_publisher')
    publisher = node.create_publisher(Twist, 'robot2/cmd_vel', 10)

    msg = Twist()
    msg.linear.x = x
    msg.angular.z = z

    for i in range(50):
        publisher.publish(msg)
        node.get_logger().info('Publishing: {}'.format(msg))
        time.sleep(0.3)

    node.destroy_node()

    return asyncio.sleep(0)
