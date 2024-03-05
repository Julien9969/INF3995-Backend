import socketio

sio = socketio.AsyncServer(cors_allowed_origins="*", async_mode="asgi")
socket_app = socketio.ASGIApp(sio)

@sio.event
async def connect(sid, env):
    print("New Client Connected to This id :" + " " + str(sid))


@sio.event
async def disconnect(sid):
    print("Client Disconnected: " + " " + str(sid))

# Websocket files must be imported at the end to avoid circular dependancy error
from backend_server.websocket.mission import sockets_mission
