from backend_server.classes.common import RobotInformation, MissionStatus, Log
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
            distance=result.distance,
            elapsedTime=result.duration,
            isSimulation=result.is_simulation,
            robotCount=result.robot_count,
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
            missionState=result.mission_state,
            distance=result.distance,
            robotCount=result.robot_count,
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
            id=robot.robot_id,
            name=robot.id,
            battery=robot.battery,
            position=robot.position,
            distance=robot.distance,
            state=robot.state,
            lastUpdate=robot.last_update,
            initialPosition=robot.initial_position,
        )
        robots.append(robot_data)
    return robots


def retrieve_map(mission_id: int) -> str:
    session = SessionLocal()
    result = session.query(MapDB).filter(MapDB.mission_id == mission_id).one_or_none()
    session.close()
    map_data = None
    if result:
        map_data = result.map_data
    return map_data


def get_new_mission_id() -> int:
    session = SessionLocal()
    result = session.query(MissionDB).count()
    session.close()
    return result + 1
