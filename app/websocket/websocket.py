from . import logger, sio


# Event handler for new client connections
@sio.event
async def connect(sid, environ):
    logger.info("Client connected", sid)
    await sio.emit('TEST', "Hello from Server!", to=sid)


@sio.on('message')
async def handle_message(sid, data):
    logger.info('Message from {}: {}'.format(sid, data))
    await sio.emit('reply', "This is a reply", to=sid)


# Event handler for client disconnections
@sio.event
async def disconnect(sid):
    logger.info("Client disconnected", sid)
