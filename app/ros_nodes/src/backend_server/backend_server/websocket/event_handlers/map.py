import math
from backend_server.websocket.base import sio
from backend_server.websocket.events import Events

import logging
from fastapi.concurrency import run_in_threadpool
import rclpy
from rclpy.node import Node

from nav_msgs.msg import OccupancyGrid
from array import array

import base64

logging.basicConfig(level=logging.DEBUG)

class MapSubscriber(Node):
    newMapAvailable = False
    def __init__(self):
        super().__init__('map_subscriber')
        self.subscription = self.create_subscription(
            OccupancyGrid,
            'robot1/map',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, occupancy_grid: OccupancyGrid):
        logging.debug(f"Map received")
        base_64_map_data = self.convertDataToBase64Str(occupancy_grid)
        self.base_64_map_img = f'data:image/bmp;base64,{base_64_map_data}'
        self.newMapAvailable = True


    def convertDataToBase64Str(self, grid):
        width, height = self.get_grid_dimensions(grid)
        width_msb, width_lsb = self.calculate_msb_lsb(width)
        height_msb, height_lsb = self.calculate_msb_lsb(height)
        logging.debug(f"Map received W: {width} H: {height}, W msb: {width_msb} W lsb: {width_lsb} ; H msb: {height_msb} H lsb: {height_lsb}")
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


class MapManager():
    mission_ongoing = False
    @staticmethod
    async def start_map_listener():
        MapManager.mission_ongoing = True
        if(not rclpy.ok()):
            rclpy.init() # will be removed later
        mapSubscriber = MapSubscriber()
        while MapManager.mission_ongoing and rclpy.ok():
            try:
                await run_in_threadpool(lambda:rclpy.spin_once(mapSubscriber, timeout_sec=4))
                if mapSubscriber.newMapAvailable and mapSubscriber.base_64_map_img is not None:
                    await send_map_image(mapSubscriber.base_64_map_img)
                    mapSubscriber.newMapAvailable = False
            except Exception as err:
                logging.debug(f"Exception in Map manager: {err}") #not important
                pass
        mapSubscriber.destroy_node()
        
    @staticmethod
    def stop_map_listener():
        MapManager.mission_ongoing = False
            
    
async def send_map_image(map_data):
    logging.debug(f"Sending map data: {map_data}")
    """
    Send the map data to the client
    """
    await sio.emit(Events.MAP_DATA.value, map_data)

def twos_comp_byte(val):
    bits = 8
    """compute the 2's complement of int value val"""
    if (val & (1 << (bits - 1))) != 0: # if sign bit is set e.g., 8bit: 128-255
        val = val - (1 << bits)        # compute negative value
    return val                         # return positive value as is
