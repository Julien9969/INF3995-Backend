from backend_server.websocket.events import Events
from backend_server.websocket.base import sio
from backend_server.api.mission.mission_base import MissionData
from backend_server.api.mission.mission_base import ROBOT_COUNT

import json
import time
import asyncio


frequency = 1  # second / 1 Hz


class StatusUpdate:
    def __init__(self):
        self.missionState = MissionData().get_mission_state()
        self.elapsedTime = MissionData().get_mission_duration()
        self.startTimestamp = MissionData().start_timestamp
        self.batteries = MissionData().get_battery()
        self.distances = MissionData().get_robot_status()
        self.count = ROBOT_COUNT
        self.timestamp = int(time.time())  # Timestamp of when the status was updated

    def to_json(self):
        return json.dumps(self.__dict__)


async def send_updates():
    """
    Emits formatted log to the client
    """
    while True:
        update = StatusUpdate()
        # JSON is used to ensure compatibility with the frontend
        await sio.emit(Events.MISSION_STATUS.value, update.to_json())
        await asyncio.sleep(frequency)
