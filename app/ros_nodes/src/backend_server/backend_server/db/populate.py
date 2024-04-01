from sqlalchemy import exists
from sqlalchemy.orm import Session
from backend_server.db.models import Mission, Robot
from backend_server.db.session import SessionLocal


def populate_db(session=SessionLocal()):
    # Add code to populate the database with initial logic
    try:
        mission_exists = session.query(exists().where(Mission.mission_id == 1)).scalar()
        if not mission_exists:
            session.add(Mission(mission_id=1, start_date=None, duration=None))

        for robot_id in [1, 0, 2]:
            robot_exists = session.query(exists().where(Robot.robot_id == 1)).scalar()
            if not robot_exists:
                session.add(Robot(robot_id=robot_id))
        session.commit()
        session.close()
    except Exception as e:
        pass