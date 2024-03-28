from pydantic import BaseModel

from backend_server.db.models.tables_models import retrieve_mission, retrieve_missions_resume

class HistoryBase:

    @staticmethod
    async def get_missions_resume() -> str:
        """
        Get the mission history from the database
        """
        # Retrieve specific information for each mission from the database using a query
        missions_resume = retrieve_missions_resume()
        return missions_resume

    @staticmethod
    async def get_complete_mission(mission_id: int) -> str:
        """
        Get the mission history from the database
        """
        mission = retrieve_mission(mission_id)
        return mission
        
    
class HistoryResponse(BaseModel):
    data: str




