from backend_server.common import MissionStatus
from backend_server.db.models import Mission, Log, Robot, Map
from backend_server.db.session import SessionLocal


def save_mission(mission: MissionStatus):
    session = SessionLocal()
    session.add(Mission(start_timestamp=mission['timestamp'],
                        duration=mission['elapsedTime'],
                        is_simulation=False))
    session.commit()
    session.close()


def save_logs(logs):
    session = SessionLocal()
    for log in logs:
        session.add(Log(timestamp=log['timestamp'],
                        eventType=log['eventType'],
                        message=log['message']))

    session.commit()
    session.close()


def save_robots(robots):
    session = SessionLocal()
    for robot in robots:
        session.add(Robot(mission_id=robot['mission_id'],
                          status=robot['status'],
                          id=robot['id']))
    session.commit()
    session.close()


def save_map(mission_id, map_data):
    session = SessionLocal()
    session.add(Map(mission_id=mission_id,
                    map_data=map_data))
    session.commit()
    session.close()
