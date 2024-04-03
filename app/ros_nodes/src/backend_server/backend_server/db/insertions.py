from backend_server.common import MissionStatus
from backend_server.db.models import Mission
from backend_server.db.session import SessionLocal


def save_mission(mission: MissionStatus):
    session = SessionLocal()
    session.add(Mission(start_timestamp=mission['timestamp'],
                        duration=mission['elapsedTime'],
                        is_simulation=False))
    session.commit()
    session.close()
