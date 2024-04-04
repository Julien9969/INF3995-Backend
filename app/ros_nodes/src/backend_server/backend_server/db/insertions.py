from backend_server.common import MissionStatus
from backend_server.db.models import Mission as MissionDB, Log as LogDB
from backend_server.db.session import SessionLocal


def save_mission(mission: MissionStatus):
    session = SessionLocal()
    session.add(MissionDB(start_timestamp=mission['timestamp'],
                          duration=mission['elapsedTime'],
                          is_simulation=False))
    session.commit()
    session.close()


def save_logs(logs):
    session = SessionLocal()
    for log in logs:
        session.add(LogDB(timestamp=log['timestamp'],
                          eventType=log['eventType'],
                          message=log['message']))

    session.commit()
    session.close()


def save_robots(robots: list[dict]):
    pass


def save_map(mission_id, map_data):
    pass
