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

from array import array

import base64

logging.basicConfig(level=logging.DEBUG)

class MapSubscriber(Node):
    newMapAvailable = False
    odom_1 = None
    odom_2 = None
    last_odom_1 = None
    last_odom_2 = None

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
            return
        base_64_map_data = self.convertDataToBase64Str(occupancy_grid)
        self.base_64_map_img = f'data:image/bmp;base64,{base_64_map_data}'
        self.newMapAvailable = True

    def odom_callback_1(self, odom: Odometry):
        logging.debug(f"== Odom robot 1 : {odom.pose.pose}")
        if self.odom_1 is None:
            self.last_odom_1 = odom.pose.pose
            self.odom_1 = odom.pose.pose
        else:
            self.last_odom_1 = self.odom_1
            self.odom_1 = odom.pose.pose
            self.update_distance(1, self.odom_1, self.last_odom_1)
        
    def odom_callback_2(self, odom: Odometry):
        logging.debug(f"== Odom robot 2 : {odom.pose.pose}")
        if self.odom_2 is None:
            self.last_odom_2 = odom.pose.pose
            self.odom_2 = odom.pose.pose
        else:
            self.last_odom_2 = self.odom_2
            self.odom_2 = odom.pose.pose
            self.update_distance(2, self.odom_2, self.last_odom_2)

    def convertDataToBase64Str(self, grid):
        width, height = self.get_grid_dimensions(grid)
        width_msb, width_lsb = self.calculate_msb_lsb(width)
        height_msb, height_lsb = self.calculate_msb_lsb(height)
        data = self.create_data_array(width, height, width_msb, width_lsb, height_msb, height_lsb, grid)
        return base64.b64encode(data).decode('utf-8')

    def get_grid_dimensions(self, grid):
        return grid.info.width, grid.info.height

    def calculate_msb_lsb(self, value): 
        # Calculer l'octet le plus significatif, et le moins significatif
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
        map_origin_x = grid.info.origin.position.x
        map_origin_y = grid.info.origin.position.y
        res = grid.info.resolution
        robot1_pos = (
            int(math.floor((self.odom_1.position.x - map_origin_x) / res)),
            height - int(math.floor((self.odom_1.position.y - map_origin_y) / res)),
        )
        robot2_pos = (
            int(math.floor((self.odom_2.position.x - map_origin_x) / res)),
            height - int(math.floor((self.odom_2.position.y - map_origin_y) / res)),
        )

        try:
            robot1 = RobotsData().get_robot(1)
            robot1.update_position(Position(x=robot1_pos[0], y=robot1_pos[1]))
            
            robot2 = RobotsData().get_robot(2)
            robot2.update_position(Position(x=robot2_pos[0], y=robot2_pos[1]))
        except Exception as err:
            logging.debug(f"--- EXCEPTION: {err}")
        for i in range(height):
            for j in range(width):
                point_value = grid.data[width*i + j]
                self.append_point_value_to_data(data, point_value)
            self.add_padding_to_data(data, width)
            
    def append_point_value_to_data(self, data, point_value):
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

    def update_distance(self, robot_num, odom, last_odom):
        d_x = odom.position.x - last_odom.position.x
        d_y = odom.position.y - last_odom.position.y
        self.distances[robot_num-1] += math.sqrt(d_x**2 + d_y**2)
        robot = RobotsData().get_robot(robot_num)
        robot.distance = self.distances[robot_num-1]

    async def log_positions_distance(self, robot_num, odom):
        await send_log(message=f"Position: x = {round(odom.position.x, 3)}, y = {round(odom.position.y, 3)}", robot_id=robot_num, event_type=LogType.SENSOR)
        await send_log(message=f"Distance parcourue: {round(self.distances[robot_num-1], 3)}", robot_id=robot_num, event_type=LogType.SENSOR)


def twos_comp_byte(val):
    bits = 8
    # Pour calculer le complement a 2 du nombre
    if (val & (1 << (bits - 1))) != 0: # if sign bit is set e.g., 8bit: 128-255
        val = val - (1 << bits)        # compute negative value
    return val                         # return positive value as is
