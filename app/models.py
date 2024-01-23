from sqlalchemy import Enum, Column, Integer, String
from .database import Base
from .classes.something_type import SomethingType


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
