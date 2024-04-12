import logging
from fastapi import APIRouter, status
from fastapi import responses, status
from .identify_base import IdentifyBase, IdentifyResponse

logging.basicConfig(level=logging.INFO)
router = APIRouter(include_in_schema=True)

@router.get("/", response_model=IdentifyResponse)
async def get_identify() -> responses.JSONResponse:
    logging.info(f"Request to identify service for robot_id:")
    return responses.JSONResponse (
        { 'data':  await IdentifyBase.launch_client() }, 
        status_code=status.HTTP_200_OK
    )

@router.get("/robot/{robot_id}")
async def get_identify_id(robot_id: int) -> responses.JSONResponse:
    logging.info(f"Request to identify service for robot_id: {robot_id}")
    result = await IdentifyBase.launch_client(robot_id)
    if(not result):
        return responses.JSONResponse ({ 'data':  "Request to identify service failed !" } , status_code=status.HTTP_404_NOT_FOUND)
    else:
        return responses.JSONResponse ({ 'data':  result }, status_code=status.HTTP_200_OK)
    
@router.get("/connected")
async def get_connected_robots() -> responses.JSONResponse:

    result = await IdentifyBase.list_connected_robot()

    if(not result):
        return responses.JSONResponse ({'data':  "Request to identify service failed !"} , status_code=status.HTTP_404_NOT_FOUND)
    else:
        return responses.JSONResponse (result, status_code=status.HTTP_200_OK)