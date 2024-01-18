from sqlalchemy import Enum, Column, Integer
from .database import Base
from .classes.something_type import SomethingType

class Something(Base):
    __tablename__ = "something"

    id = Column(Integer, primary_key=True)
    # other_id = Column(Integer, ForeignKey("othertable.id"), nullable=False)
    truc = Column(Enum(SomethingType), nullable=False)

