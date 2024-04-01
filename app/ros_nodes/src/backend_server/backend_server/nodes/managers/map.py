import logging

import rclpy
from backend_server.common import WebsocketsEvents
from backend_server.websocket.emitter import send_raw
from fastapi.concurrency import run_in_threadpool
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node

logging.basicConfig(level=logging.DEBUG)


class MapManager:
    missionOngoing = False

    @staticmethod
    async def start_map_listener():
        MapManager.missionOngoing = True
        if not rclpy.ok():
            rclpy.init()
        mapPublisher = MapPublisher()
        while MapManager.missionOngoing:
            try:
                await run_in_threadpool(lambda: rclpy.spin_once(mapPublisher, timeout_sec=4))
                if mapPublisher.newMapAvailable and mapPublisher.base_64_map_img is not None:
                    await send_raw(WebsocketsEvents.MAP_DATA, mapPublisher.base_64_map_img)
                    mapPublisher.newMapAvailable = False
            except Exception as err:
                logging.debug(f"Exception in Map manager: {err}")

    @staticmethod
    def stop_map_listener():
        MapManager.missionOngoing = False



