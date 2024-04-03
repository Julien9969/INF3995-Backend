import asyncio
import json
import logging
import time

from backend_server.common import Log, LogType
from backend_server.common import WebsocketsEvents, MissionStatus
from backend_server.logic.mission import Mission
from backend_server.websocket.base import sio

from ..db.session import SessionLocal

FREQUENCY = 1  # second / 1 Hz


async def send(event: WebsocketsEvents, data: dict = None, sid=None):
    """
    Emits the event to the clients
    """
    json_data = json.dumps(data)
    await sio.emit(event.value, json_data, to=sid)


async def send_raw(event: WebsocketsEvents, data, sid=None):
    """
    Emits the event to the clients
    """
    await sio.emit(event.value, data, to=sid)


async def send_log(message: str, robot_id=2, event_type=LogType.LOG):
    """
    Emits formatted log to the clients
    """
    log = Log(message=message,
              timestamp=int(time.time()),
              robotId=robot_id,
              eventType=event_type,
              missionId=1)

    await send(WebsocketsEvents.LOG_DATA, log)


async def send_mission_updates():
    """
    Emits formatted log to the client
    """
    while True:
        mission = Mission()
        update = mission.get_status()
        await send(WebsocketsEvents.MISSION_STATUS, update)
        await asyncio.sleep(FREQUENCY)


async def send_robot_updates():
    """
    Emits formatted log to the client
    """
    # TODO: read from the result
    pass
