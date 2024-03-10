from pydantic import BaseModel

# from datetime import datetime
from backend_server.schemas.something_type import SomethingType


class SomethingBase(BaseModel):
    id: int
    truc: SomethingType
    # value: float
    # timestamp: datetime

    class Config:
        orm_mode = True


class File(BaseModel):
    name: str
    id: int
    content: str

class FileObject():
    def __init__(self, name: str, id: int, content: str):
        self.name = name
        self.id = id
        self.content = content

    name: str
    id: int
    content: str
    robotId: int

class FileId(BaseModel):
    name: str
    id: int
    robotId: int    

class serviceError():
    def __init__(self, message: str, content: str):
        self.message = message
        self.content = content

    message: str
    content: str


