from pydantic import BaseModel
# from datetime import datetime
from .classes.something_type import SomethingType


class SomethingBase(BaseModel):
    id: int
    truc: SomethingType
    # value: float
    # timestamp: datetime

    class Config:
        orm_mode = True