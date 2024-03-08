from backend_server.websocket.events import Events
from backend_server.websocket.base import sio

import json
import time

from enum import Enum


class LogType(Enum):
    LOG = "log"
    COMMAND = "command"
    SENSOR = "sensor"


class Log:
    def __init__(self, message, robotId, logType: LogType):
        self.message = message
        self.timestamp = int(time.time())
        self.robotId = robotId if robotId is not None else 1  # Usually, either 1 or 2
        self.eventType = logType.value

    def to_json(self):
        return json.dumps(self.__dict__)


async def send_log(message: str, robot_id=1, event_type=LogType.LOG):
    """
    Emits formatted log to the clients
    """
    log = Log(message, robot_id, event_type)
    # JSON is used to ensure compatibility with the frontend
    await sio.emit(Events.LOG_DATA.value, log.to_json())
