from pydantic import BaseModel


class File(BaseModel):
    name: str
    id: int
    content: str


class FileObject:
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
