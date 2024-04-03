import json

from fastapi import APIRouter
from fastapi import responses, status

from .mission_history_base import HistoryBase, HistoryResponse

router = APIRouter(include_in_schema=True)


@router.get("/", response_model=HistoryResponse)
async def get_missions() -> responses.JSONResponse:
    result = await HistoryBase.get_missions_resume()
    json_result = json.dumps(result)
    if not result:
        return responses.JSONResponse({'logic': "Request to History service failed !"},
                                      status_code=status.HTTP_404_NOT_FOUND)
    else:
        return responses.JSONResponse(json_result, status_code=status.HTTP_200_OK)


@router.get("/{mission_id}")
async def get_mission(mission_id: int) -> responses.JSONResponse:
    result = await HistoryBase.get_complete_mission(mission_id)
    json_result = json.dumps(result)
    if not result:
        return responses.JSONResponse({'logic': "Request to History service failed !"},
                                      status_code=status.HTTP_404_NOT_FOUND)
    else:
        return responses.JSONResponse(json_result, status_code=status.HTTP_200_OK)
