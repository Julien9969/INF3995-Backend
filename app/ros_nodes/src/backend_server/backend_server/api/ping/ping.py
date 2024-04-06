from pydantic import BaseModel


class PingBase:

    @staticmethod
    def ping():
        return "pong"


class PingResponse(BaseModel):
    data: str
