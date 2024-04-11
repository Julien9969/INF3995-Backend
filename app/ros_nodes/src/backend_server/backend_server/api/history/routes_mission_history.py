import json

from fastapi import APIRouter
from fastapi import responses, status

from .mission_history_base import HistoryBase, HistoryResponse

router = APIRouter(include_in_schema=True)


@router.get("/")
async def get_mission() -> responses.JSONResponse:
    result = await HistoryBase.get_missions()
    json_result = json.dumps(result)
    if not result:
        return responses.JSONResponse({},
                                      status_code=status.HTTP_404_NOT_FOUND)
    else:
        return responses.JSONResponse(json_result, status_code=status.HTTP_200_OK)


@router.get("/status/{mission_id}")
async def get_mission(mission_id: int) -> responses.JSONResponse:
    result = await HistoryBase.get_mission(mission_id)
    json_result = json.dumps(result)
    if not result:
        return responses.JSONResponse({},
                                      status_code=status.HTTP_404_NOT_FOUND)
    else:
        return responses.JSONResponse(json_result, status_code=status.HTTP_200_OK)


@router.get("/map/{mission_id}")
async def get_mission(mission_id: int) -> responses.PlainTextResponse:
    map_data = await HistoryBase.get_map(mission_id)
    if not map_data:
        return responses.PlainTextResponse("",
                                           status_code=status.HTTP_404_NOT_FOUND)
    else:
        return responses.PlainTextResponse(map_data, status_code=status.HTTP_200_OK)


@router.get("/robots/{mission_id}")
async def get_mission(mission_id: int) -> responses.JSONResponse:
    result = await HistoryBase.get_robots(mission_id)
    json_result = json.dumps(result)
    if not result:
        return responses.JSONResponse({},
                                      status_code=status.HTTP_404_NOT_FOUND)
    else:
        return responses.JSONResponse(json_result, status_code=status.HTTP_200_OK)


@router.get("/logs/{mission_id}")
async def get_mission(mission_id: int) -> responses.JSONResponse:
    result = await HistoryBase.get_logs(mission_id)
    json_result = json.dumps(result)
    if not result:
        return responses.JSONResponse({},
                                      status_code=status.HTTP_404_NOT_FOUND)
    else:
        return responses.JSONResponse(json_result, status_code=status.HTTP_200_OK)
