import logging
import socketio

logger = logging.getLogger(__name__)

sio = socketio.AsyncServer(logger=True,
                           engineio_logger=True,
                           async_mode='asgi',
                           cors_allowed_origins='*')

logger.info("Module loaded")
