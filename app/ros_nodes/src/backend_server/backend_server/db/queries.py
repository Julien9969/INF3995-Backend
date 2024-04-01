from backend_server.common import RobotInformation, MissionStatus
from backend_server.db.models import Mission, Robot, Log
from backend_server.db.session import SessionLocal


def retrieve_missions_resume():
    # Add code to retrieve mission history from the database
    session = SessionLocal()
    results = session.query(
        Mission.mission_id,
        Mission.start_date,
        Mission.duration,
        Robot.robot_id).all(
    )

    # Process the results
    missions_data = {}
    for mission_id, start_date, duration, robot_id in results:
        if mission_id not in missions_data:
            missions_data[mission_id] = {
                'mission_id': mission_id,
                'start_date': start_date,
                'duration': duration,
                'robots': []
            }
        missions_data[mission_id]['robots'].append({'robot_id': robot_id, })
    session.close()
    return missions_data


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
    # Add code to retrieve logs for a specific mission from the database
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
    result = session.query(Mission).get(mission_id)
    assert str(result).startswith("data:image/bmp;base64,"), "Invalid image data"
    session.close()
    return result