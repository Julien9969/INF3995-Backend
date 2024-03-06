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


class EventType(Enum):
    LOG = "log"
    COMMAND = "command"
    SENSOR = "sensor"


class Log:
    def __init__(self, message, robotId, eventType: EventType):
        self.message = message
        self.timestamp = int(time.time())
        self.robotId = robotId if robotId is not None else 1  # Usually, either 1 or 2
        self.eventType = eventType
        #TODO ajouter le parametre missionId

    def to_json(self):
        return json.dumps(self.__dict__)


async def send_log(message, robotId):
    print(message)
    """
    """
    log = Log(message, robotId, eventType=EventType.LOG)
    if DEBUG:
        logging.debug(log.to_json())
        
    #Envoi vers la database
    new_mission = Mission()
    new_robot = Robot(robotId=log.robotId)
    new_log = LogDB(mission=new_mission, robot=new_robot, log_type='INFO', message='Mission started successfully')

    SessionLocal.add_all([new_mission, new_robot, new_log])
    SessionLocal.commit()
    await sio.emit(MissionEvents.LOG_DATA, log.to_json())
    log = Log(message, 1, EventType.LOG)
    await sio.emit(MissionEvents.LOG_DATA.value, log.to_json())

