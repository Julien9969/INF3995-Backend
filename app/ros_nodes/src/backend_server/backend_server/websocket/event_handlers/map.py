from backend_server.websocket.base import sio
from backend_server.websocket.events import Events


@sio.on(Events.MAP_DATA.value)
async def send_map_data(sid):
    """
    Send the map data to the client
    """
    await sio.emit(Events.MAP_DATA.value, to=sid)