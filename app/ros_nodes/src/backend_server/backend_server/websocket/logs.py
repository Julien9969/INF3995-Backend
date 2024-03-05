import logging

from backend_server.websocket.events import MissionEvents
from backend_server.websocket.base import sio

import json
import time

from enum import Enum


class EventType(Enum):
    LOG = "log"
    COMMAND = "command"
    SENSOR = "sensor"


class Log:
    def __init__(self, message, robotId, eventType: EventType):
        self.message = message
        self.timestamp = int(time.time())
        self.robotId = robotId if robotId is not None else 1  # Usually, either 1 or 2
        self.eventType = eventType.value

    def to_json(self):
        return json.dumps(self.__dict__)


async def send_log(message):
    """
    """
    log = Log(message, 1, EventType.LOG)
    await sio.emit(MissionEvents.LOG_DATA.value, log.to_json())

