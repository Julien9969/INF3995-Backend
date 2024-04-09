from backend_server.classes.common import MissionStatus, RobotInformation, Log
from backend_server.db.models import Mission as MissionDB, Log as LogDB, Map as MapDB, Robot as RobotDB
from backend_server.db.session import SessionLocal

import time


def init_missions():
    session = SessionLocal()
    mission = MissionDB(
        start_timestamp=int(time.time()),
        duration=300,
        is_simulation=False,
        robot_count=2,
        distance=0.9,
        mission_state='ENDED',
    )
    session.add(mission)
    robot = RobotDB(
        mission_id=mission.id,
        state='IDLE',
        distance=0,
        battery=0,
        position='{x: 200, y: 200}',
        initial_position='{x: 100, y: 100}',
        last_update=int(time.time()),
        robot_id=1,
    )
    session.add(robot)
    session.commit()
    session.close()


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
                          is_simulation=False))

    for robot in robots:
        session.add(RobotDB(mission_id=mission['missionId'],
                            state=robot['state'],
                            distance=robot['distance'],
                            battery=robot['battery'],
                            position=robot['position'],  # json
                            initial_position=robot['initialPosition'],  # json
                            last_update=robot['lastUpdate'],
                            robot_id=robot['id']))

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
