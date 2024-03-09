import logging
from ..db.models.exemples_models import Mission, Robot
from ..db.models.exemples_models import Log as LogDB


from ..db.session import SessionLocal

from .events import  MissionEvents
from .base import sio
from backend_server.websocket.events import MissionEvents
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
        self.logType = logType
        #TODO ajouter le parametre missionId

    def to_json(self):
        return json.dumps(self.__dict__)


async def send_log(message, robotId):
    print(message)
    """
    """
    log = Log(message, robotId, logType=LogType.LOG)
        
    #Envoi vers la database
    # new_mission = Mission()
    # new_robot = Robot(robot_id=log.robotId)
    new_log = LogDB(mission_id=6, robot_id=log.robotId, log_type='INFO', message=log.message)

    session = SessionLocal()

    session.add_all([new_log])
    session.commit()
    log = Log(message, robotId, LogType.LOG.value)
    await sio.emit(MissionEvents.LOG_DATA.value, log.to_json())

