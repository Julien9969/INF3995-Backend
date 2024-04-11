import asyncio
import json
import time

from backend_server.classes.common import Log, LogType, WebsocketsEvents
from backend_server.models.mission import Mission
from backend_server.models.robots import RobotsData
from backend_server.websocket.base import sio

FREQUENCY = 1  # second / 1 Hz


async def send(event: WebsocketsEvents, data: dict = None, sid=None):
    json_data = json.dumps(data)
    await sio.emit(event.value, json_data, to=sid)


async def send_raw(event: WebsocketsEvents, data, sid=None):
    await sio.emit(event.value, data, to=sid)


async def send_log(message: str, robot_id=2, event_type=LogType.LOG):
    log = Log(message=message,
              timestamp=int(time.time()),
              robotId=robot_id,
              eventType=event_type,
              missionId=1)

    await send(WebsocketsEvents.LOG_DATA, log)


async def send_map_image(map_data):
    """
    Called by the node at an undermined frequency
    """
    await sio.emit(WebsocketsEvents.MISSION_MAP.value, map_data)


async def send_mission_updates():
    while True:
        mission = Mission()
        update = mission.get_status()
        await send(WebsocketsEvents.MISSION_STATUS, update)
        await asyncio.sleep(FREQUENCY)


async def send_robot_updates():
    while True:
        robots = RobotsData()
        update = robots.get_robots()
        await send(WebsocketsEvents.ROBOT_STATUS, update)
        await asyncio.sleep(FREQUENCY)
