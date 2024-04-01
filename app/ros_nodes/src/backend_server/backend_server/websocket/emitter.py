import asyncio
import json
import time

from backend_server.db.models import Log as LogDB
from backend_server.common import Log, LogType
from backend_server.common import WebsocketsEvents, MissionStatus
from backend_server.logic.mission import MissionData
from backend_server.websocket.base import sio

from ..db.session import SessionLocal

FREQUENCY = 1  # second / 1 Hz


async def send(event: WebsocketsEvents, data: dict = None, sid=None):
    """
    Emits the event to the clients
    """
    json_data = json.dumps(data)
    await sio.emit(event, json_data, to=sid)


async def send_map(event: WebsocketsEvents, map, sid=None):
    """
    Emits the event to the clients
    """
    await sio.emit(event.value, map, to=sid)


async def send_log(message: str, robot_id=2, event_type=LogType.LOG):
    """
    Emits formatted log to the clients
    """
    log = Log(message=message,
              timestamp=int(time.time()),
              robotId=robot_id,
              eventType=event_type,
              missionId=1)

    # Creating and sending of the log to the Database
    new_log_row = LogDB(mission_id=log.missionId, robot_id=log.robotId, log_type=log.eventType, message=log.message)
    session = SessionLocal()
    session.add_all([new_log_row])
    session.commit()
    # Sending the log to the frontend
    await send(WebsocketsEvents.LOG_DATA.value, log)


async def send_updates():
    """
    Emits formatted log to the client
    """
    while True:
        mission_data = MissionData()
        update = MissionStatus(missionState=mission_data.state,
                               timestamp=int(time.time()),
                               elapsed=int(time.time()) - mission_data.start_timestamp,
                               isSimulation=False,
                               startTimestamp=mission_data.start_timestamp
                               )
        # JSON is used to ensure compatibility with the frontend
        await send(WebsocketsEvents.MISSION_STATUS.value, update)
        await asyncio.sleep(FREQUENCY)
