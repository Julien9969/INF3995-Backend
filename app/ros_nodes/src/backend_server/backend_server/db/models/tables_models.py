from sqlalchemy import Enum, Column, Integer, String, ForeignKey
from sqlalchemy.orm import relationship
from backend_server.db.session import Base, SessionLocal
from backend_server.schemas.something_type import SomethingType


class Something(Base):
    __tablename__ = "something"

    id = Column(Integer, primary_key=True)
    # other_id = Column(Integer, ForeignKey("othertable.id"), nullable=False)
    truc = Column(Enum(SomethingType), nullable=False)


class stringTable(Base):
    # should be lowercase because with capital letters the name of table will have double quotes (select * from "stringTable"; will be the query)
    __tablename__ = "stringtable" 

    id = Column(Integer, primary_key=True)
    txt = Column(String, nullable=False)

# Define the Log model
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

# Define the Mission model
class Mission(Base):
    __tablename__ = 'missions'

    mission_id = Column(Integer, primary_key=True)
    # Add other mission-related columns as needed
    logs = relationship('Log', back_populates='mission')

# Define the Robot model
class Robot(Base):
    __tablename__ = 'robots'

    robot_id = Column(Integer, primary_key=True)
    # Add other robot-related columns as needed
    logs = relationship('Log', back_populates='robot')

def populate_db(session=SessionLocal()):
    # Add code to populate the database with initial data
    try:
        # Check if mission with mission_id=1 exists
        if not session.query(Mission).filter(Mission.mission_id == 1).first():
            session.add(Mission(mission_id=1))

        # Check if robots with robot_id=1, 0, 2 exist
        for robot_id in [1, 0, 2]:
            if not session.query(Robot).filter(Robot.robot_id == robot_id).first():
                session.add(Robot(robot_id=robot_id))
        session.commit()
        session.close()
        print("Database populated")
    except Exception as e:
        pass
