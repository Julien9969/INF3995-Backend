from backend_server.db.session import Base
from sqlalchemy import Column, Integer, String, ForeignKey, Boolean, Float


class Mission(Base):
    __tablename__ = 'missions'

    id = Column(Integer, primary_key=True)
    start_timestamp = Column(Integer)
    duration = Column(Integer)
    is_simulation = Column(Boolean)
    robot_count = Column(Integer)
    distance = Column(Float)
    mission_state = Column(String)  # see MissionState


class Log(Base):
    __tablename__ = 'logs'

    id = Column(Integer, primary_key=True)
    timestamp = Column(Integer)
    event_type = Column(String)
    message = Column(String)
    robot_id = Column(Integer, ForeignKey('robots.id'))
    mission_id = Column(Integer, ForeignKey('missions.id'))


class Robot(Base):
    __tablename__ = 'robots'

    mission_id = Column(Integer, ForeignKey('missions.id'))
    state = Column(String)  # see RobotState
    distance = Column(Float)
    battery = Column(Float)
    position = Column(String)  # json of x, y
    initial_position = Column(String)  # json of x, y
    last_update = Column(Integer)
    robot_id = Column(Integer)
    id = Column(Integer, primary_key=True)


class Map(Base):
    __tablename__ = 'maps'

    id = Column(Integer, primary_key=True)
    mission_id = Column(Integer, ForeignKey('missions.id'))
    map_data = Column(String)
