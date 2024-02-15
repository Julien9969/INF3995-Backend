import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Twist


class SpinRobot(Node):

    def __init__(self):
        super().__init__('spin_robot')
        self.publisher_ = self.create_publisher(Twist, 'limo/cmd_vel', 10)
        # timer_period = 0.5  # seconds
        # self.timer = self.create_timer(timer_period, self.timer_callback)
        # self.i = 0

    def trigger(self):
        rotate_msg = Twist()
        # rotate_msg.linear.x = 1.0
        rotate_msg.angular.z = 3.14
        print(rotate_msg)
        
        self.publisher_.publish(rotate_msg)
        # self.publisher_.publish("{linear: {x: 1, y: 0, z: 0}, angular: {x: 0, y: 0, z: 6.28}}")

        self.get_logger().info('Spinning: "%s"' % rotate_msg)

    def stop(self):
        rotate_msg = Twist()        
        self.publisher_.publish(rotate_msg)
        self.get_logger().info('Stopping spinning: "%s"' % rotate_msg)
        # self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = SpinRobot()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()