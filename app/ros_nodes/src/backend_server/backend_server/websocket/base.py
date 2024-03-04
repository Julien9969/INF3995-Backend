import socketio

sio = socketio.AsyncServer(cors_allowed_origins="*", async_mode="asgi")
socket_app = socketio.ASGIApp(sio)
is_mission_ongoing = False


@sio.event
async def connect(sid, env):
    print("New Client Connected to This id :" + " " + str(sid))


@sio.event
async def disconnect(sid):
    print("Client Disconnected: " + " " + str(sid))


@sio.on("mission-start")
async def set_mission_start(sid):
    global is_mission_ongoing
    is_mission_ongoing = True
    await sio.emit("mission-start")


@sio.on("mission-end")
async def set_mission_end(sid):
    global is_mission_ongoing
    is_mission_ongoing = False
    await sio.emit("mission-end")


@sio.on("mission-status")
async def get_mission_status(sid):
    await sio.emit("mission-status", is_mission_ongoing, to=sid)
