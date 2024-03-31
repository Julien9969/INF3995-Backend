import logging

from backend_server.api.mission.mission_base import start_mission, stop_mission
from backend_server.websocket.base import sio
from backend_server.websocket.event_handlers.map import MapManager
from backend_server.common import WebsocketsEvents, MissionStatus
from backend_server.websocket.logs import send_log

from .logs_mission import LogManager


@sio.on(WebsocketsEvents.MISSION_STATUS.value)
async def get_mission_status(sid):
    """
    Inform a newly joined client of the current mission status
    """
    status = MissionStatus()
    assert status is not None
    await sio.emit(WebsocketsEvents.MISSION_STATUS.value, status.to_json(), to=sid)


@sio.on(WebsocketsEvents.MISSION_START.value)
async def set_mission_start(sid, _=None):
    """
    Confirm to clients that the mission has been started
    """
    logging.debug("Starting mission")
    result = start_mission()
    await sio.emit(WebsocketsEvents.MISSION_START.value)
    await send_log(f"Robots response to start: {result}")
    await LogManager.start_record_logs()


@sio.on(WebsocketsEvents.MISSION_MAP.value)
async def start_record_map(sid, _=None):
    """
    Start sending map to client
    """
    logging.debug("Map")
    await sio.emit(WebsocketsEvents.MISSION_MAP.value)
    await send_log(f"Start receive map")
    await MapManager.start_map_listener()


@sio.on(WebsocketsEvents.MISSION_END.value)
async def set_mission_end(sid, _=None):
    """
    Confirm to clients that the mission has been stopped
    """
    result = stop_mission()
    await send_log(f"Robots response to stop: {result}")
    await sio.emit(WebsocketsEvents.MISSION_END.value)
    LogManager.stop_record_logs()
    MapManager.stop_map_listener()
