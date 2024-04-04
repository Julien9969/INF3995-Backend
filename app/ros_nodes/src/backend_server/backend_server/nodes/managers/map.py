import logging

from backend_server.nodes.subscribers.map import MapSubscriber
import rclpy
from backend_server.common import WebsocketsEvents
from backend_server.websocket.emitter import send_map_image, send_raw
from fastapi.concurrency import run_in_threadpool


class MapManager:
    mission_ongoing = False
    @staticmethod
    async def start_map_listener():
        logging.debug("starting map listener")
        MapManager.mission_ongoing = True
        map_subscriber = MapSubscriber()
        while MapManager.mission_ongoing and rclpy.ok():
            try:
                await run_in_threadpool(lambda:rclpy.spin_once(map_subscriber, timeout_sec=4))
                if map_subscriber.newMapAvailable and map_subscriber.base_64_map_img is not None:
                    await send_map_image(map_subscriber.base_64_map_img)
                    map_subscriber.newMapAvailable = False
            except Exception as err:
                logging.debug(f"Exception in Map manager: {err}") #not important
                pass
        map_subscriber.destroy_node()
        
    @staticmethod
    def stop_map_listener():
        MapManager.mission_ongoing = False
