from .logs_mission import record_logs
from backend_server.websocket.base import sio
from backend_server.websocket.events import Events
from backend_server.api.mission.mission_base import  MissionData, start_mission, stop_mission
from backend_server.websocket.logs import send_log, LogType

import json


@sio.on(Events.MISSION_STATUS.value)
async def get_mission_status(sid):
    """
    Inform a newly joined client of the current mission status
    """
    status = MissionData().get_mission_state()
    assert status is not None
    await sio.emit(Events.MISSION_STATUS.value, json.dumps(status), to=sid)


@sio.on(Events.MISSION_START.value)
async def set_mission_start(sid):
    """
    Confirm to clients that the mission has been started
    """
    result = start_mission()
    assert result is not None
    await sio.emit(Events.MISSION_START.value)
    await send_log(f"Robots response to start: {result}")
    await record_logs()


@sio.on(Events.MISSION_END.value)
async def set_mission_end(sid):
    """
    Confirm to clients that the mission has been stopped
    """
    result = stop_mission()
    assert result is not None
    await send_log(f"Robots response to stop: {result}")
    await sio.emit(Events.MISSION_END.value)

