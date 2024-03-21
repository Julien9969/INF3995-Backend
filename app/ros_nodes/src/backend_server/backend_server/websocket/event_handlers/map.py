import math
from backend_server.websocket.base import sio
from backend_server.websocket.events import Events

import logging
from fastapi.concurrency import run_in_threadpool
import rclpy
from rclpy.node import Node

from nav_msgs.msg import OccupancyGrid
from array import array
import asyncio

import base64

# from io import BytesIO
# from PIL import Image
# pip install Pillow !!!!!!!!!!! not PIL which is deprecated

logging.basicConfig(level=logging.DEBUG)

class MapPublisher(Node):
    newMapAvailable = False
    def __init__(self):
        super().__init__('map_publisher')
        self.subscription = self.create_subscription(
            OccupancyGrid,
            'map',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, occupancy_grid: OccupancyGrid):
        logging.debug("============= map received in backend")
        # logging.debug(occupancy_grid.data)
        base_64_map_data = self.convertDataToBase64Str(occupancy_grid)
        # logging.debug(base_64_map_data)
        self.base_64_map_img = f'data:image/bmp;base64,{base_64_map_data}'
        self.newMapAvailable = True
        logging.debug("============= map received in backend OUT")

    def convertDataToBase64Str(self, grid):
        logging.debug("============= map backend 0")
        width = grid.info.width
        height = grid.info.height
        height_msb = math.floor(height/256)
        height_lsb = height % 256
        width_msb = math.floor(width/256)
        width_lsb = width % 256
        if width_lsb > 127:
            width_lsb = twos_comp_byte(width_lsb)
        if height_lsb > 127:
            height_lsb = twos_comp_byte(height)
        logging.debug(f"============= map backend W: {width} H: {height}, ---- W msb: {width_msb} W lsb: {width_lsb} ; H msb: {height_msb} H lsb: {height_lsb}")
        try:
            data = array('b', [0x42, 0x4D, twos_comp_byte(0xBA), twos_comp_byte(0xA5), 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x36, 0x00, 0x00, 0x00, 0x28, 0x00, 0x00, 0x00, width_lsb, width_msb, 0x00, 0x00, height_lsb, height_msb, 0x00, 0x00, 0x01, 0x00, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, twos_comp_byte(0x84), twos_comp_byte(0xA5), 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        except Exception as err:
            logging.debug(f"============= CRASH {err}")

        # data = []
        logging.debug("============= map backend 1")
        for i in range(height):
            for j in range(width):
                point_value = grid.data[width*i + j]
                if(point_value == -1):
                    # format = BGR , donc voici du rose
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
                # data.append(twos_comp_byte(0xff)) # pas de transparence! uniquement BGR
            # end of a row, add padding
            pad_n = 4 - ((width * 3) % 4)
            if pad_n == 4:
                pad_n = 0
            for _ in range(pad_n):
                data.append(0)
        # logging.debug(f"============= pad_n = {pad_n}")


        # pil_img = Image.fromarray(img)
        # buff = BytesIO()
        # pil_img.save(buff, format="JPEG")
        # new_image_string = base64.b64encode(buff.getvalue()).decode("utf-8")


        # data_bytes = bytes(data)
        # logging.debug("=== data_bytes: ")
        # logging.debug(data)
        return base64.b64encode(data).decode('utf-8')


class MapManager():
    missionOngoing = False
    @staticmethod
    async def start_map_listener():
        MapManager.missionOngoing = True
        logging.debug("===== starting mission")
        if(not rclpy.ok()):
            rclpy.init()
        mapPublisher = MapPublisher()
        while MapManager.missionOngoing:
            try:
                await run_in_threadpool(lambda:rclpy.spin_once(mapPublisher, timeout_sec=4))
                # await asyncio.sleep(1)
                logging.debug("=== node finished spinning map pub")
                if mapPublisher.newMapAvailable and mapPublisher.base_64_map_img is not None:
                    await send_map_image(mapPublisher.base_64_map_img)
                    logging.debug("=== map sent by ws!")
                    mapPublisher.newMapAvailable = False
            except Exception as err:
                logging.debug(f"Exception in Map manager: {err}")
        # mapPublisher.destroy_node()
        
    @staticmethod
    def stop_map_listener():
        MapManager.missionOngoing = False
            
    
async def send_map_image(map_data):
    """
    Send the map data to the client
    """
    logging.debug("=== sending map image by ws...")
    await sio.emit(Events.MAP_DATA.value, map_data)

def twos_comp_byte(val):
    bits = 8
    """compute the 2's complement of int value val"""
    if (val & (1 << (bits - 1))) != 0: # if sign bit is set e.g., 8bit: 128-255
        val = val - (1 << bits)        # compute negative value
    return val                         # return positive value as is
