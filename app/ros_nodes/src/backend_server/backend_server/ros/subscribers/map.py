import math
from backend_server.websocket.emitter import send_log
from backend_server.classes.common import LogType, Position
from backend_server.models.robots import RobotsData

from backend_server.websocket.base import sio

import logging
from fastapi.concurrency import run_in_threadpool
import rclpy
from rclpy.node import Node

from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import Pose

from array import array

import base64

logging.basicConfig(level=logging.DEBUG)

class MapSubscriber(Node):
    newMapAvailable = False
    def __init__(self):
        super().__init__('map_subscriber')
        self.subscription_odom_1 = self.create_subscription(
            Odometry,
            'robot1/odom',
            self.odom_callback_1,
            10)
        self.subscription_odom_1  # prevent unused variable warning

        self.subscription_odom_2 = self.create_subscription(
            Odometry,
            'robot2/odom',
            self.odom_callback_2,
            10)
        self.subscription_odom_2  # prevent unused variable warning

        self.subscription = self.create_subscription(
            OccupancyGrid,
            'map',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.distances = [0, 0]

    def listener_callback(self, occupancy_grid: OccupancyGrid):
        logging.debug(f"Map received")
        if (self.odom_1 is None or self.odom_2 is None):
            logging.debug(f"Odom not received yet!! cannot do map")
        base_64_map_data = self.convertDataToBase64Str(occupancy_grid)
        self.base_64_map_img = f'data:image/bmp;base64,{base_64_map_data}'
        self.newMapAvailable = True

    def odom_callback_1(self, odom: Odometry):
        logging.debug(f"== Odom robot 1 : {odom.pose.pose}")
        self.last_odom_1 = self.odom_1
        self.odom_1 = odom.pose.pose
        if self.last_odom_1 is None:
            robot = RobotsData().get_robot(1)
            robot.initial_position = Position(self.odom_1.position.x, self.odom_1.position.y)
        else:
            self.log_positions_distance(1, self.odom_1, self.last_odom_1)
        
    def odom_callback_2(self, odom: Odometry):
        logging.debug(f"== Odom robot 2 : {odom.pose.pose}")
        self.last_odom_2 = self.odom_2
        self.odom_2 = odom.pose.pose
        if self.last_odom_2 is None:
            robot = RobotsData().get_robot(2)
            robot.initial_position = Position(self.odom_2.position.x, self.odom_2.position.y)
        else:
            self.log_positions_distance(2, self.odom_2, self.last_odom_2)

    def convertDataToBase64Str(self, grid):
        width, height = self.get_grid_dimensions(grid)
        width_msb, width_lsb = self.calculate_msb_lsb(width)
        height_msb, height_lsb = self.calculate_msb_lsb(height)
        # logging.debug(f"Map received W: {width} H: {height}, W msb: {width_msb} W lsb: {width_lsb} ; H msb: {height_msb} H lsb: {height_lsb}")
        logging.debug(f"========================= MAP ANALYSIS:")
        logging.debug(f"===== grid.info.resolution : {grid.info.resolution}")
        logging.debug(f"===== W-H : {grid.info.width}-{grid.info.height}")
        logging.debug(f"===== position : {grid.info.origin.position}")
        logging.debug(f"===== orientation : {grid.info.origin.orientation}")
        data = self.create_data_array(width, height, width_msb, width_lsb, height_msb, height_lsb, grid)
        return base64.b64encode(data).decode('utf-8')

    def get_grid_dimensions(self, grid):
        return grid.info.width, grid.info.height

    def calculate_msb_lsb(self, value): # calculates the most significant byte and the least significant byte of a value
        msb = math.floor(value/256)
        lsb = value % 256
        if lsb > 127:
            lsb = twos_comp_byte(lsb)
        return msb, lsb

    def create_data_array(self, width, height, width_msb, width_lsb, height_msb, height_lsb, grid):
        data = []
        try:
            data = array('b', [0x42, 0x4D, twos_comp_byte(0xBA), twos_comp_byte(0xA5), 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x36, 0x00, 0x00, 0x00, 0x28, 0x00, 0x00, 0x00, width_lsb, width_msb, 0x00, 0x00, height_lsb, height_msb, 0x00, 0x00, 0x01, 0x00, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, twos_comp_byte(0x84), twos_comp_byte(0xA5), 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        except Exception as err:
            logging.debug(f"Crashed while transforming map to image: {err}")
        self.append_grid_data_to_array(data, width, height, grid)
        return data

    def append_grid_data_to_array(self, data, width, height, grid):
        logging.debug(f"=== WIDTH {width} , HEIGHT {height}")
        map_origin_x = grid.info.origin.position.x
        map_origin_y = grid.info.origin.position.y
        res = grid.info.resolution
        robot1_pos = (
            math.floor((self.odom_1.position.y - map_origin_y) / res),
            math.floor((self.odom_1.position.x - map_origin_x) / res),
        )
        robot2_pos = (
            math.floor((self.odom_2.position.y - map_origin_y) / res),
            math.floor((self.odom_2.position.x - map_origin_x) / res),
        )
        logging.debug(f"--- robot pos on MAP: {robot1_pos, robot2_pos}")

        robot1 = RobotsData().get_robot(1)
        robot1.update_position(Position(robot1_pos[0], robot1_pos[1]))
        
        robot2 = RobotsData().get_robot(2)
        robot2.update_position(Position(robot2_pos[0], robot2_pos[1]))

        for i in range(height):
            for j in range(width):
                point_value = grid.data[width*i + j]
                # Testing robot localisation
                robot_here = 0
                # if (abs(robot1_pos[0] - i) < 8 and abs(robot1_pos[1] - j) < 8):
                #     robot_here = 1
                # elif (abs(robot2_pos[0] - i) < 4 and abs(robot2_pos[1] - j) < 4):
                #     robot_here = 2
                #
                self.append_point_value_to_data(data, point_value, robot_here)
            self.add_padding_to_data(data, width)
            
    def append_point_value_to_data(self, data, point_value, robot_here):
        if robot_here == 1:
            data.append(twos_comp_byte(254))
            data.append(0)
            data.append(70)            
            return
        if robot_here == 2:
            data.append(70)            
            data.append(0)
            data.append(twos_comp_byte(254))
            return
        if point_value == -1:
            # format = BGR , donc voici du gris pour les zones non explorées
            data.append(twos_comp_byte(175))
            data.append(twos_comp_byte(175))
            data.append(twos_comp_byte(175))
        else:
            point_color = math.floor((point_value)/100*255)
            point_color = 255 - point_color #invert colors for black walls and white empty
            if point_color > 127:
                point_color = twos_comp_byte(point_color)
            data.append(point_color)
            data.append(point_color)
            data.append(point_color)

    def add_padding_to_data(self, data, width):
        # end of a row, add padding because BMP must have a multiple of 4
        pad_n = 4 - ((width * 3) % 4)
        if pad_n == 4:
            pad_n = 0
        for _ in range(pad_n):
            data.append(0)

    def log_positions_distance(self, robot_num, odom, last_odom):
        d_x = odom.position.x - last_odom.position.x
        d_y = odom.position.y - last_odom.position.y
        self.distances[robot_num-1] += math.sqrt(d_x**2 + d_y**2)
        send_log(f"Position: {odom}", robot_id=robot_num, event_type=LogType.SENSOR)
        send_log(f"Distance totale parcourue: {self.distances[robot_num-1]}", robot_id=robot_num, event_type=LogType.SENSOR)
        robot = RobotsData().get_robot(robot_num)
        robot.distance = self.distances[robot_num-1]



def twos_comp_byte(val):
    bits = 8
    """compute the 2's complement of int value val"""
    if (val & (1 << (bits - 1))) != 0: # if sign bit is set e.g., 8bit: 128-255
        val = val - (1 << bits)        # compute negative value
    return val                         # return positive value as is
