from backend_server.common import RobotInformation, MissionStatus
from backend_server.db.models import Mission, Robot, Map
from backend_server.db.session import SessionLocal


def retrieve_missions_resume():
    session = SessionLocal()
    results = session.query(
        Mission.id,
        Mission.start_timestamp,
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
        mission_data = MissionStatus(
            mission_id=result.id,
            start_timestamp=result.start_timestamp,
            duration=result.duration,
            is_simulation=result.is_simulation
        )
    else:
        mission_data = None

    session.close()
    return mission_data


def retrieve_robot_status(mission_id: int) -> list[RobotInformation]:
    session = SessionLocal()
    result = session.query(Robot).filter(Robot.id == mission_id).all()
    session.close()
    return result


def retrieve_map_data(mission_id: int) -> str:
    session = SessionLocal()
    result = session.query(Map).get(Map.id == mission_id)
    # TODO: could maybe also save size of image in db
    assert str(result).startswith("data:image/bmp;base64,"), "Invalid image data"
    session.close()
    return result


def get_new_mission_id() -> int:
    session = SessionLocal()
    result = session.query(Mission).count()
    session.close()
    return result + 1
