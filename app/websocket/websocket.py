import socketio
import logging

sio = socketio.AsyncServer(async_mode='asgi', cors_allowed_origins='*')

logging.getLogger('socketio').setLevel(logging.INFO)


@sio.event
async def connect(sid, environ):
    print("Connected", sid)
    await sio.emit('reply', "Connect Event", to=sid)


@sio.on('*')
async def handle_message(sid, data):
    print(f"Message from {sid}: {data}")
    await sio.emit('reply', data, to=sid)


@sio.event
async def disconnect(sid):
    print("Disconnected", sid)
