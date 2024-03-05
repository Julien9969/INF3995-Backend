from backend_server.websocket.base import sio
from backend_server.websocket.events import MissionEvents
from backend_server.api.mission.mission_base import MissionBase


@sio.on(MissionEvents.MISSION_STATUS.value)
async def get_mission_status(sid):
    """
    Inform a newly joined client of the current mission status
    """
    status = MissionBase.is_mission_ongoing()
    await sio.emit(MissionEvents.MISSION_STATUS.value, status, to=sid)


@sio.on(MissionEvents.MISSION_START.value)
async def set_mission_start(sid):
    """
    Confirm to clients that the mission has been started
    """
    result = MissionBase.start_mission()
    print(result)
    await sio.emit(MissionEvents.LOG_DATA.value, result)
    await sio.emit(MissionEvents.MISSION_START.value)


@sio.on(MissionEvents.MISSION_END.value)
async def set_mission_end(sid):
    """
    Confirm to clients that the mission has been stopped
    """
    result = MissionBase.stop_mission()
    await send_log(result)
    await sio.emit(MissionEvents.MISSION_END.value)
