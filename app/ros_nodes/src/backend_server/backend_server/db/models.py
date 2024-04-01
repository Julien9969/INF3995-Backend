from backend_server.db.session import Base, SessionLocal
from sqlalchemy import Column, Integer, String, ForeignKey, Interval, DateTime
from sqlalchemy.orm import relationship


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
