import logging

from backend_server.nodes.clients.mission import Mission
from backend_server.websocket.base import sio
from backend_server.nodes.managers.map import MapManager
from backend_server.nodes.managers.logs import LogManager
from backend_server.common import WebsocketsEvents, MissionStatus
from backend_server.websocket.emitter import send, send_log
from backend_server.nodes.clients.robot import IdentifyBase


@sio.event
async def connect(sid, environ):
    """
    Confirm to clients that the connection has been established
    """
    await send(WebsocketsEvents.CONNECT)


@sio.event
async def disconnect(sid):
    """
    Confirm to clients that the connection has been closed
    """
    await send(WebsocketsEvents.DISCONNECT)


@sio.on(WebsocketsEvents.MISSION_START.value)
async def set_mission_start(sid, _=None):
    """
    Confirm to clients that the mission has been started
    """
    mission = Mission()
    result = mission.start_mission()
    await send_log(f"Robots response to start: {result}")
    await MapManager.start_map_listener()
    await LogManager.start_record_logs()


@sio.on(WebsocketsEvents.MISSION_END.value)
async def set_mission_end(sid, _=None):
    """
    Confirm to clients that the mission has been stopped
    """
    mission = Mission()
    result = mission.stop_mission()
    await send_log(f"Robots response to stop: {result}")
    LogManager.stop_record_logs()
    MapManager.stop_map_listener()


@sio.on(WebsocketsEvents.IDENTIFY_REQUEST.value)
async def identify_request(sid, robot_id):
    """
    Confirm to clients that the mission has been aborted
    """
    logging.info("Identify request received")
    result = await IdentifyBase.launch_client(robot_id)
    await send(WebsocketsEvents.IDENTIFY_REQUEST.value, result)


@sio.on(WebsocketsEvents.HEADBACKBASE_REQUEST.value)
async def headbackbase_request():
    """
    Confirm to clients that the mission has been aborted
    """
    logging.info("Headbackbase request received")
    # TODO: call Node or wtv
