from backend_server.websocket.base import sio
from backend_server.websocket.mission.mission_events import MissionEvents

@sio.on(MissionEvents.TEST_EVENT.value)
async def client_side_receive_msg(sid, msg):
    print("Msg receive from " +str(sid) +"and msg is : ",str(msg))
    await sio.emit("event", "marioo time")

@sio.on(MissionEvents.MISSION_STATUS.value)
async def get_mission_status(sid):
    await sio.emit(MissionEvents.MISSION_STATUS.value, is_mission_ongoing, to=sid)


is_mission_ongoing = False

@sio.on(MissionEvents.MISSION_START.value)
async def set_mission_start(sid):
    global is_mission_ongoing
    is_mission_ongoing = True
    await sio.emit(MissionEvents.MISSION_START.value)


@sio.on(MissionEvents.MISSION_END.value)
async def set_mission_end(sid):
    global is_mission_ongoing
    is_mission_ongoing = False
    await sio.emit(MissionEvents.MISSION_END.value)