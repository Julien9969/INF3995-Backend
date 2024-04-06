from backend_server.common import RobotInformation, MissionStatus, Log
from backend_server.db.models import Mission as MissionDB, Robot as RobotDB, Map as MapDB, Log as LogDB
from backend_server.db.session import SessionLocal


def retrieve_missions():
    session = SessionLocal()
    results = session.query(MissionDB).all()
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


def retrieve_logs(mission_id: int) -> list:
    session = SessionLocal()
    results = session.query(LogDB).filter(LogDB.mission_id == mission_id).all()
    session.close()
    logs = []
    for result in results:
        log = Log(
            message=result.message,
            timestamp=result.timestamp,
            robotId=result.robot_id,
            eventType=result.event_type,
            missionId=result.mission_id,
        )
        logs.append(log)
    return logs


def retrieve_mission(mission_id: int) -> MissionStatus:
    session = SessionLocal()
    result = session.query(MissionDB).get(mission_id)
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


def retrieve_robots(mission_id: int) -> list[RobotInformation]:
    session = SessionLocal()
    result = session.query(RobotDB).filter(RobotDB.mission_id == mission_id).all()
    session.close()
    robots = []
    for robot in result:
        robot_data = RobotInformation(
            id=robot.id,
            name=robot.id,
            battery=robot.battery,
            position=robot.position,
            distance=robot.distance,
            state=robot.status,
            lastUpdate=robot.last_update,
            initialPosition=robot.initial_position,
        )
        robots.append(robot_data)
    return robots


def retrieve_map(mission_id: int) -> str:
    session = SessionLocal()
    result = session.query(MapDB).get(mission_id)
    session.close()
    map_data = None
    if result:
        map_data = result.map_data
        assert str(map_data).startswith("data:image/bmp;base64,"), "Invalid image data"
    return map_data


def get_new_mission_id() -> int:
    session = SessionLocal()
    result = session.query(MissionDB).count()
    session.close()
    return result + 1
