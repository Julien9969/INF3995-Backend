from sqlalchemy import Enum, Column, Integer, String, ForeignKey, Interval, DateTime, exists
from sqlalchemy.orm import relationship
from backend_server.db.session import Base, SessionLocal
from backend_server.schemas.something_type import SomethingType
from datetime import datetime, timedelta

class Log(Base):
    __tablename__ = 'logs'

    log_id = Column(Integer, primary_key=True)
    mission_id = Column(Integer, ForeignKey('missions.mission_id'))
    robot_id = Column(Integer, ForeignKey('robots.robot_id'))
    log_type = Column(String)
    message = Column(String)

    # Define relationships with Mission and Robot tables
    mission = relationship('Mission', back_populates='logs')
    robot = relationship('Robot', back_populates='logs')

class Mission(Base):
    __tablename__ = 'missions'

    mission_id = Column(Integer, primary_key=True)
    start_date = Column(DateTime)
    duration = Column(Interval)
    logs = relationship('Log', back_populates='mission')


class Robot(Base):
    __tablename__ = 'robots'

    robot_id = Column(Integer, primary_key=True)
    logs = relationship('Log', back_populates='robot')

def populate_db(session=SessionLocal()):
    # Add code to populate the database with initial data
    try:
        mission_exists = session.query(exists().where(Mission.mission_id == 1)).scalar()
        if not mission_exists:
            session.add(Mission(mission_id=1, start_date= None, duration= None))

        for robot_id in [1, 0, 2]:
            robot_exists = session.query(exists().where(Robot.robot_id == 1)).scalar()
            if not robot_exists:
                session.add(Robot(robot_id=robot_id))
        session.commit()
        session.close()
        print("Database populated")
    except Exception as e:
        pass

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
        missions_data[mission_id]['robots'].append({'robot_id': robot_id,})
    session.close()
    return missions_data

def retrieve_mission(mission_id_to_retrieve):
    # Add code to retrieve a specific mission from the database
    session = SessionLocal()
    result = session.query(Mission).get(mission_id_to_retrieve)

    # Process the result to make it JSON serializable
    if result:
        mission_data = {
            "mission_id": result.mission_id,
            "start_date": result.start_date.isoformat() if result.start_date else None,
            "duration": str(result.duration) if result.duration else None,
            # Include other mission attributes as needed
        }
        # Include logs associated with the mission
        mission_data["logs"] = [
            {
                "log_id": log.log_id,
                "robot_id": log.robot_id,
                "log_type": log.log_type,
                "message": log.message,
                # Include other log attributes as needed
            }
            for log in result.logs
        ]
    else:
        mission_data = None

    session.close()
    return mission_data
