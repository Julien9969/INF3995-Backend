import asyncio
import time

from backend_server.api.mission.mission_base import MissionData
from backend_server.common import WebsocketsEvents, MissionStatus
from backend_server.websocket.base import sio

FREQUENCY = 1  # second / 1 Hz


async def send_updates():
    """
    Emits formatted log to the client
    """
    while True:
        mission_data = MissionData()
        update = MissionUpdate(missionState=mission_data.state,
                               timestamp=int(time.time()),
                               elapsed=int(time.time()) - mission_data.start_timestamp,
                               isSimulation=False,
                               startTimestamp=mission_data.start_timestamp
                               )
        # JSON is used to ensure compatibility with the frontend
        await sio.emit(WebsocketsEvents.MISSION_STATUS.value, update.to_json())
        await asyncio.sleep(FREQUENCY)
