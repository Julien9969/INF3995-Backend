import logging

from backend_server.models.mission import Mission
from backend_server.websocket.base import sio
from backend_server.ros.managers.map import MapManager
from backend_server.ros.managers.logs import LogManager
from backend_server.classes.common import WebsocketsEvents
from backend_server.websocket.emitter import send, send_log
# from backend_server.ros.clients.robots import IdentifyBase


@sio.on(WebsocketsEvents.MISSION_START.value)
async def set_mission_start(sid, _=None):
    """
    Confirm to clients that the mission has been started
    """
    mission = Mission()
    mission.start_mission()
    logging.info("Mission started")
    await LogManager.start_record_logs()

@sio.on(WebsocketsEvents.MISSION_MAP.value)
async def start_record_map(sid, _=None):
    """
    Start sending map to client
    """
    await sio.emit(WebsocketsEvents.MISSION_MAP.value)
    await MapManager.start_map_listener()

@sio.on(WebsocketsEvents.MISSION_END.value)
async def set_mission_end(sid, _=None):
    """
    Confirm to clients that the mission has been stopped
    """
    Mission().stop_mission()
    logging.info("Mission ended")
    LogManager.stop_record_logs()
    MapManager.stop_map_listener()


@sio.on(WebsocketsEvents.IDENTIFY_REQUEST.value)
async def identify_request(sid, robot_id):
    """
    Confirm to clients that the mission has been aborted
    """
    logging.info("Identify request received")
    # TODO: result = await IdentifyBase.launch_client(robot_id)
    await send(WebsocketsEvents.IDENTIFY_FEEDBACK, "result")


@sio.on(WebsocketsEvents.HEADBACKBASE_REQUEST.value)
async def headbackbase_request():
    """
    Confirm to clients that the mission has been aborted
    """
    logging.info("Headbackbase request received")
    # TODO: call Node or wtv
