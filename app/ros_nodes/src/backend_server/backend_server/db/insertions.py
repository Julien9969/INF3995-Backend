from backend_server.common import MissionStatus
from backend_server.db.models import Mission
from backend_server.db.session import SessionLocal
from sqlalchemy import exists


def save_mission(mission: MissionStatus):
    session = SessionLocal()
    mission_exists = session.query(exists().where(Mission.mission_id == mission['missionId'])).scalar()
    if not mission_exists:
        session.add(Mission(mission_id=mission['missionId'],
                            start_date=mission['timestamp'],
                            duration=mission['elapsedTime']))
    session.commit()
    session.close()
