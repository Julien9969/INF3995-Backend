from backend_server.websocket.base import sio
from backend_server.websocket.events import Events

import logging
from rclpy.node import Node

from nav_msgs import OccupancyGrid

import base64

class MapPublisher(Node):

    def __init__(self):
        super().__init__('map_publisher')
        self.subscription = self.create_subscription(
            OccupancyGrid,
            'map',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, occupancy_grid: OccupancyGrid):
        logging.debug("map received in backend")
        base_64_map_data = self.convertDataToBase64Str(occupancy_grid.data)
        base_64_map_img = f'data:image/bmp;base64,{base_64_map_data}'
        self.send_map_image(base_64_map_img)

    def convertDataToBase64Str(data):
        data_bytes = bytes(data)
        return base64.b64encode(data_bytes).decode('utf-8')

    async def send_map_image(map_data):
        """
        Send the map data to the client
        """
        await sio.emit(Events.MAP_DATA.value, map_data)
