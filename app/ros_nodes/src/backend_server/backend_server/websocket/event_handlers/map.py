from backend_server.websocket.base import sio
from backend_server.websocket.events import Events

import logging
from fastapi.concurrency import run_in_threadpool
import rclpy
from rclpy.node import Node

from nav_msgs.msg import OccupancyGrid

import base64

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
        base_64_map_data = self.convertDataToBase64Str(occupancy_grid.data)
        self.base_64_map_img = f'data:image/bmp;base64,{base_64_map_data}'
        self.newMapAvailable = True
        logging.debug("============= map received in backend OUT")

    def convertDataToBase64Str(data):
        data_bytes = bytes(data)
        return base64.b64encode(data_bytes).decode('utf-8')

class MapManager():
    missionOngoing = True
    @staticmethod
    async def start_map_listener():
        logging.debug("===== starting mission")
        if(not rclpy.ok()):
            rclpy.init()
        mapPublisher = MapPublisher()
        while MapManager.missionOngoing:
            try:
                await run_in_threadpool(lambda:rclpy.spin_once(mapPublisher, timeout_sec=2))
                logging.debug("=== node finished spinning map pub")
                if mapPublisher.newMapAvailable and mapPublisher.base_64_map_img is not None:
                    await send_map_image(mapPublisher.base_64_map_img)
                    logging.debug("=== map sent by ws!")
                    mapPublisher.newMapAvailable = False
            except KeyboardInterrupt:
                pass
        # mapPublisher.destroy_node()
            
    
async def send_map_image(map_data):
    """
    Send the map data to the client
    """
    logging.debug("=== sending map image by ws...")
    await sio.emit(Events.MAP_DATA.value, map_data)

