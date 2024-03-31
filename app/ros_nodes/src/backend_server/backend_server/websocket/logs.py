from ..db.models.tables_models import Log as LogDB
from ..db.session import SessionLocal
from .base import sio
from backend_server.websocket.base import sio
from backend_server.common import Log, LogType, WebsocketsEvents


async def send_log(message: str, robot_id=2, event_type=LogType.LOG):
    """
    Emits formatted log to the clients
    """
    log = Log(robotId=robot_id, eventType=event_type, message=message)
        
    # Creating and sending of the log to the Database
    new_log_row = LogDB(mission_id=log.missionId, robot_id=log.robotId, log_type=log.eventType, message=log.message)
    session = SessionLocal()
    session.add_all([new_log_row])
    session.commit()
    # Sending the log to the frontend
    await sio.emit(WebsocketsEvents.LOG_DATA.value, log.to_json())

