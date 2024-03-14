from fastapi import APIRouter, status
from fastapi import responses, status
from .mission_base import MissionBase

router = APIRouter(include_in_schema=True)


@router.post("/start")
async def post_start_mission() -> responses.JSONResponse:
    

    # TODO create mission singleton object so if we call start again it will not create a new mission object
    MissionBase.start_mission()

    return responses.JSONResponse (
        { 'data':  "Mission start sent !" }, 
        status_code=status.HTTP_200_OK )

@router.post("/stop")
async def post_start_mission() -> responses.JSONResponse:

    # TODO destroy mission singleton object
    MissionBase.stop_mission()
    
    return responses.JSONResponse (
        { 'data':  "Mission stop sent !" }, 
        status_code=status.HTTP_200_OK )

@router.get("/status")
async def get_mission_status() -> responses.JSONResponse:

    missionStatus = MissionBase.get_mission_status()

    return responses.JSONResponse (
        missionStatus,
        status_code=status.HTTP_200_OK )