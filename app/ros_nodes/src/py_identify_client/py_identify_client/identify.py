import time
from interfaces.srv import Identify

import sys
import rclpy
from rclpy.node import Node
from .spin_robot import SpinRobot


class IdentifyService(Node):

    def __init__(self):
        super().__init__('identify_service')
        self.srv = self.create_service(Identify, 'identify', self.serve)

    def serve(self, request, response):
        response.b = request.a * 2
        self.get_logger().info(f'Incoming request, a: {request.a}')

        SpinRobot().trigger()
        time.sleep(10)
        SpinRobot().stop()

        return response


def main():
    rclpy.init()

    identify_service = IdentifyService()

    rclpy.spin(identify_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()