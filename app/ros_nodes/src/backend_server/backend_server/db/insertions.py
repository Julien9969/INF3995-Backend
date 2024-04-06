from backend_server.common import MissionStatus, RobotInformation, Log
from backend_server.db.models import Mission as MissionDB, Log as LogDB, Map as MapDB, Robot as RobotDB
from backend_server.db.session import SessionLocal


def save_mission(mission: MissionStatus, robots: list[RobotInformation], logs: list[Log], map_data: str):
    total_distance = 0.0
    robot_count = len(robots)
    for robot in robots:
        total_distance += robot['distance']

    session = SessionLocal()

    session.add(MissionDB(start_timestamp=mission['startTimestamp'],
                          duration=mission['elapsedTime'],
                          distance=total_distance,
                          robot_count=robot_count,
                          mission_state=mission['missionState'],
                          mission_id=mission['missionId'],
                          is_simulation=False))

    for robot in robots:
        session.add(RobotDB(mission_id=mission['missionId'],
                            status=robot['status'],
                            distance=robot['distance'],
                            battery=robot['battery'],
                            position=robot['position'],  # json
                            initial_position=robot['initialPosition'],  # json
                            last_update=robot['lastUpdate'],
                            robot_id=robot['robotId']))

    for log in logs:
        session.add(LogDB(timestamp=log['timestamp'],
                          mission_id=mission['missionId'],
                          eventType=log['eventType'],
                          robot_id=log['robotId'],
                          message=log['message']))

    session.add(MapDB(mission_id=mission['missionId'],
                      map_data=map_data))

    session.commit()
    session.close()

