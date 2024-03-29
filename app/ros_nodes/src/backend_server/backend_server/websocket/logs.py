import logging

from backend_server.api.mission.mission_base import MissionData
from ..db.models.tables_models import Log as LogDB


from ..db.session import SessionLocal

from .base import sio
from backend_server.websocket.events import Events
from backend_server.websocket.base import sio

import json
import time

from enum import Enum


class LogType(Enum):
    LOG = "log"
    COMMAND = "command"
    BATTERY = "battery"
    SENSOR = "sensor"
    COORDS = "coords"


class Log:
    def __init__(self, message: str, robotId:int , logType : LogType= LogType.LOG, missionId = 1):
        self.message = message
        self.timestamp = int(time.time())
        self.robotId = robotId if robotId is not None else 1  # Usually, either 1 or 2
        self.eventType = logType.value
        self.missionId = missionId

    def to_json(self):
        return json.dumps(self.__dict__)


async def send_log(message: str, robot_id=2, event_type=LogType.LOG):
    """
    Emits formatted log to the clients
    """
    log = Log(message, robot_id, event_type)
        
    # Creating and sending of the log to the Database
    new_log_row = LogDB(mission_id=log.missionId, robot_id=log.robotId, log_type=log.eventType, message=log.message)
    session = SessionLocal()
    session.add_all([new_log_row])
    session.commit()
    # Sending the log to the frontend
    await sio.emit(Events.LOG_DATA.value, log.to_json())

def update_battery(message: str, robot_id=2):
    """
    Emits formatted battery log to the clients
    """
    # Extract battery level from the message
    battery_level = message.split(":")[1].strip().replace("%", "")
    logging.debug(battery_level)

    # Get the singleton instance of MissionData
    mission_data = MissionData()

    # Ensure the batteries list is long enough
    while len(mission_data.batteries) < robot_id:
        mission_data.batteries.append(0)

    # Update the battery level
    mission_data.batteries[robot_id - 1] = int(battery_level)