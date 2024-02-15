from sqlalchemy import Enum, Column, Integer, String
from app.db.session import Base
from app.schemas.something_type import SomethingType


class Something(Base):
    __tablename__ = "something"

    id = Column(Integer, primary_key=True)
    # other_id = Column(Integer, ForeignKey("othertable.id"), nullable=False)
    truc = Column(Enum(SomethingType), nullable=False)

class Mission(Base):
    __tablename__ = "mission"
    missionId = Column(Integer, primary_key = True)
    date = Column(String, nullable=False)
    duration = Column(String, nullable =False)
    totalDistance = Column(Integer, nullable = False)

class Robot(Base):
    __tablename__ = "robot"
    robotId = Column(Integer, primary_key = True)
    robotName = Column(String, nullable=False)


class stringTable(Base):
    # should be lowercase because with capital letters the name of table will have double quotes (select * from "stringTable"; will be the query)
    __tablename__ = "stringtable" 

    id = Column(Integer, primary_key=True)
    txt = Column(String, nullable=False)
