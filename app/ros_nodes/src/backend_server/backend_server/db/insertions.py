from backend_server.classes.common import MissionStatus, RobotInformation, Log
from backend_server.db.models import Mission as MissionDB, Log as LogDB, Map as MapDB, Robot as RobotDB
from backend_server.db.session import SessionLocal

import time


def init_missions():
    session = SessionLocal()
    if session.query(MissionDB).count() == 0:
        mission = MissionDB(
            start_timestamp=int(time.time()),
            duration=300,
            is_simulation=False,
            robot_count=2,
            distance=0.9,
            mission_state='ENDED',
        )
        session.add(mission)
        session.commit()

        if session.query(RobotDB).count() == 0:
            robot1 = RobotDB(
                mission_id=1,
                state='IDLE',
                distance=0.1,
                battery=35,
                position='{x: 40, y: 120}',
                initial_position='{x: 40, y: 120}',
                last_update=int(time.time()),
                robot_id=1,
            )
            robot2 = RobotDB(
                mission_id=1,
                state='IDLE',
                distance=0.1,
                battery=35,
                position='{x: 100, y: 25}',
                initial_position='{x: 100, y: 25}',
                last_update=int(time.time()),
                robot_id=2,
            )
            session.add(robot1)
            session.add(robot2)
            session.commit()

        if session.query(LogDB).count() == 0:
            log1 = LogDB(
                timestamp=int(time.time()),
                mission_id=1,
                event_type='log',
                robot_id=1,
                message='Robot 1 battery level: 35%',
            )
            log2 = LogDB(
                timestamp=int(time.time()),
                mission_id=1,
                event_type='log',
                robot_id=2,
                message='Robot 2 battery level: 35%',
            )
            session.add(log1)
            session.add(log2)
            session.commit()

        if session.query(MapDB).count() == 0:
            map_data = MapDB(
                mission_id=1,
                map_data='map_data'
            )
            session.add(map_data)

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
                          is_simulation=mission['is_simulation']))
    session.commit()

    for robot in robots:
        session.add(RobotDB(mission_id=mission['missionId'],
                            state=robot['state'],
                            distance=robot['distance'],
                            battery=robot['battery'],
                            position=robot['position'],  # json
                            initial_position=robot['initialPosition'],  # json
                            last_update=robot['lastUpdate'],
                            robot_id=robot['id']))
        session.commit()

    for log in logs:
        session.add(LogDB(timestamp=log['timestamp'],
                          mission_id=mission['missionId'],
                          eventType=log['eventType'],
                          robot_id=log['robotId'],
                          message=log['message']))
        session.commit()

    session.add(MapDB(mission_id=mission['missionId'],
                      map_data=map_data))

    session.commit()
    session.close()
