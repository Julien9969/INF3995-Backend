from backend_server.websocket.events import Events
from backend_server.websocket.base import sio
from backend_server.api.mission.mission_base import MissionData

import json
import time
import asyncio

frequency = 1  # second / 1 Hz


class StatusUpdate:
    def __init__(self):
        self.mission_status = MissionData().get_mission_state()

        self.mission_duration = MissionData().get_mission_duration()

        self.robot_battery = []
        for robot_id in range(1, 3):
            self.robot_battery.append(MissionData().get_battery(robot_id))

        self.robot_distance = []
        for robot_id in range(1, 3):
            self.robot_distance.append(MissionData().get_robot_status(robot_id))

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
        print("Status Update Sent")
        await asyncio.sleep(frequency)
