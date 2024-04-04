from backend_server.common import RobotInformation, MissionStatus
from backend_server.db.models import Mission, Robot, Map, Log
from backend_server.db.session import SessionLocal


def retrieve_missions_resume():
    session = SessionLocal()
    results = session.query(Mission).all()
    session.close()
    status = []
    for result in results:
        mission = MissionStatus(
            missionId=result.id,
            startTimestamp=result.start_timestamp,
            elapsedTime=result.duration,
            isSimulation=result.is_simulation,
            robotCount=0,
            missionState='ENDED',
        )
        status.append(mission)
    return status


def retrieve_logs() -> list:
    session = SessionLocal()
    results = session.query(Log).filter(Log.mission_id == 1).all()
    session.close()
    return results


def retrieve_mission(mission_id: int) -> MissionStatus:
    session = SessionLocal()
    result = session.query(Mission).get(mission_id)
    if result:
        mission_data = MissionStatus(
            missionId=result.id,
            startTimestamp=result.start_timestamp,
            elapsedTime=result.duration,
            isSimulation=result.is_simulation,
            missionState='ENDED',
            robotCount=0,
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
