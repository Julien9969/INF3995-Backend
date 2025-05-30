import asyncio

import socketio

sio = socketio.AsyncServer(cors_allowed_origins="*", async_mode="asgi", logger=False, engineio_logger=False)
socket_app = socketio.ASGIApp(sio)

# Websocket files must be imported at the end to avoid circular dependency error
from backend_server.websocket import listener  # noqa: F401 Important to import this file to start the listeners
from backend_server.websocket.emitter import send_mission_updates, send_robot_updates

# With mission start, start sending status updates to the clients
asyncio.ensure_future(send_mission_updates())
asyncio.ensure_future(send_robot_updates())
