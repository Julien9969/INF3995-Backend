import logging

import rclpy
from backend_server.common import WebsocketsEvents
from backend_server.nodes.publishers.map import MapPublisher
from backend_server.websocket.emitter import send_raw
from fastapi.concurrency import run_in_threadpool


class MapManager:
    missionOngoing = False

    @staticmethod
    async def start_map_listener():
        MapManager.missionOngoing = True
        if not rclpy.ok():
            rclpy.init()
        map_publisher = MapPublisher()
        while MapManager.missionOngoing:
            try:
                await run_in_threadpool(lambda: rclpy.spin_once(map_publisher, timeout_sec=4))
                if map_publisher.newMapAvailable and map_publisher.base_64_map_img is not None:
                    await send_raw(WebsocketsEvents.MAP_DATA, map_publisher.base_64_map_img)
                    map_publisher.newMapAvailable = False
            except Exception as err:
                pass

    @staticmethod
    def stop_map_listener():
        MapManager.missionOngoing = False
