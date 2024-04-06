from backend_server.db.session import Base, SessionLocal
from sqlalchemy import Column, Integer, String, ForeignKey, Boolean, Float
from sqlalchemy.orm import relationship


class Mission(Base):
    __tablename__ = 'missions'

    id = Column(Integer, primary_key=True)
    start_timestamp = Column(Integer)
    duration = Column(Integer)
    is_simulation = Column(Boolean)
    robots = relationship('Robot')


class Log(Base):
    __tablename__ = 'logs'

    id = Column(Integer, primary_key=True)
    timestamp = Column(Integer)
    event_type = Column(Integer)
    message = Column(String)
    robot_id = Column(Integer, ForeignKey('robots.id'))
    mission_id = Column(Integer, ForeignKey('missions.id'))


class Robot(Base):
    __tablename__ = 'robots'

    mission_id = Column(Integer, ForeignKey('missions.id'), primary_key=True)
    status = Column(String)  # see RobotState
    distance = Column(Float)
    battery = Column(Float)
    position = Column(String)  # json of x, y
    initial_position = Column(String)  # json of x, y
    last_update = Column(Integer)
    id = Column(Integer)


class Map(Base):
    __tablename__ = 'maps'

    id = Column(Integer, primary_key=True)
    mission_id = Column(Integer, ForeignKey('missions.id'))
    map_data = Column(String)  # TODO: Big Enough Base 64 encoded image
