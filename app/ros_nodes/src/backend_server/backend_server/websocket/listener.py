import logging

from backend_server.nodes.mission import Mission
from backend_server.websocket.base import sio
from backend_server.nodes.map import MapManager
from backend_server.nodes.logs import LogManager
from backend_server.common import WebsocketsEvents, MissionStatus
from backend_server.websocket.emitter import send, send_log


@sio.on(WebsocketsEvents.MISSION_START.value)
async def set_mission_start(sid, _=None):
    """
    Confirm to clients that the mission has been started
    """
    result = Mission.start_mission()
    await send_log(f"Robots response to start: {result}")
    await MapManager.start_map_listener()
    await LogManager.start_record_logs()


@sio.on(WebsocketsEvents.MISSION_END.value)
async def set_mission_end(sid, _=None):
    """
    Confirm to clients that the mission has been stopped
    """
    result = Mission.stop_mission()
    await send_log(f"Robots response to stop: {result}")
    LogManager.stop_record_logs()
    MapManager.stop_map_listener()
