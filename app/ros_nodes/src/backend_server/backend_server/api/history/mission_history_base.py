from backend_server.db.queries import retrieve_mission, retrieve_missions, retrieve_map, retrieve_robots, retrieve_logs
from pydantic import BaseModel


class HistoryBase:

    @staticmethod
    async def get_missions():
        return retrieve_missions()

    @staticmethod
    async def get_mission(mission_id: int):
        return retrieve_mission(mission_id)

    @staticmethod
    async def get_map(mission_id: int):
        return retrieve_map(mission_id)

    @staticmethod
    async def get_robots(mission_id: int):
        return retrieve_robots(mission_id)

    @staticmethod
    async def get_logs(mission_id: int):
        return retrieve_logs(mission_id)


class HistoryResponse(BaseModel):
    data: str
