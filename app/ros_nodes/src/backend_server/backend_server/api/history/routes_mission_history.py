from fastapi import APIRouter, status
from fastapi import responses, status
from .mission_history_base import HistoryBase, HistoryResponse
import asyncio


router = APIRouter(include_in_schema=True)

@router.get("/", response_model=HistoryResponse)
async def get_missions() -> responses.JSONResponse:
    return responses.JSONResponse (
        { 'data':  await HistoryBase.get_missions_resume() }, 
        status_code=status.HTTP_200_OK
    )

@router.get("/id/{mission_id}")
async def get_mission(mission_id: int) -> responses.JSONResponse:
    result = await HistoryBase.get_complete_mission(mission_id)
    if(not result):
        return responses.JSONResponse ({ 'data':  "Request to History service failed !" } , status_code=status.HTTP_404_NOT_FOUND)
    else:
        return responses.JSONResponse ({ 'data':  result }, status_code=status.HTTP_200_OK)