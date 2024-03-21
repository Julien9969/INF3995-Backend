import asyncio
import logging
import threading
from .logs_mission import LogManager
from backend_server.websocket.base import sio
from backend_server.websocket.events import Events
from backend_server.api.mission.mission_base import start_mission, stop_mission
from backend_server.websocket.logs import send_log, LogType, Log
from backend_server.websocket.status import StatusUpdate
from backend_server.websocket.event_handlers.map import MapManager

@sio.on(Events.MISSION_STATUS.value)
async def get_mission_status(sid):
    """
    Inform a newly joined client of the current mission status
    """
    status = StatusUpdate()
    assert status is not None
    await sio.emit(Events.MISSION_STATUS.value, status.to_json(), to=sid)

@sio.on(Events.MISSION_START.value)
async def set_mission_start(sid, _=None):
    """
    Confirm to clients that the mission has been started
    """
    logging.debug("Starting mission")
    result = start_mission()
    await sio.emit(Events.MISSION_START.value)
    await send_log(f"Robots response to start: {result}")
    await LogManager.start_record_logs()

@sio.on(Events.MISSION_MAP.value)
async def start_record_map(sid, _=None):
    """
    Start sending map to client
    """
    logging.debug("Map")
    result = start_mission()
    await sio.emit(Events.MISSION_MAP.value)
    await send_log(f"Start receive map: {result}")
    await MapManager.start_map_listener()



@sio.on(Events.MISSION_END.value)
async def set_mission_end(sid, _=None):
    """
    Confirm to clients that the mission has been stopped
    """
    result = stop_mission()
    await send_log(f"Robots response to stop: {result}")
    await sio.emit(Events.MISSION_END.value)
    LogManager.stop_record_logs()
    MapManager.stop_map_listener()
    
# Debug: notify the frontend that it should
# reset the mission state because state was lost during hot reload
sio.emit(Events.MISSION_RESTART.value)
