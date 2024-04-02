from backend_server.common import RobotInformation, MissionStatus
from backend_server.db.models import Mission, Robot, Log, Map
from backend_server.db.session import SessionLocal
import json


def retrieve_missions_resume():
    session = SessionLocal()
    results = session.query(
        Mission.mission_id,
        Mission.start_date,
        Mission.duration,
        Robot.robot_id).all()

    session.close()
    return results


def retrieve_history() -> list:
    pass


def retrieve_mission(mission_id: int) -> MissionStatus:
    session = SessionLocal()
    result = session.query(Mission).get(mission_id)

    if result:
        # TODO: use data structures available in common
        mission_data = {
            "mission_id": result.mission_id,
            "start_date": result.start_date.isoformat() if result.start_date else None,
            "duration": str(result.duration) if result.duration else None,
            # Include other mission attributes as needed
        }
    else:
        mission_data = None

    session.close()
    return mission_data


def retrieve_mission_logs(mission_id: int) -> list[Log]:
    session = SessionLocal()
    result = session.query(Log).filter(Log.mission_id == mission_id).all()
    session.close()
    return result


def retrieve_robot_status(mission_id: int) -> list[RobotInformation]:
    session = SessionLocal()
    result = session.query(Robot).filter(Robot.mission_id == mission_id).all()
    session.close()
    return result


def retrieve_map_data(mission_id: int) -> str:
    session = SessionLocal()
    result = session.query(Map).get(Map.mission_id == mission_id)
    # TODO: could maybe also save size of image in db
    assert str(result).startswith("data:image/bmp;base64,"), "Invalid image data"
    session.close()
    return result


def get_new_mission_id() -> int:
    session = SessionLocal()
    result = session.query(Mission).count()
    session.close()
    return result + 1
