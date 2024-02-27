from fastapi import APIRouter, status
from fastapi import responses, status
from .navigate_base import NavigateBase, NavigateResponse


router = APIRouter(include_in_schema=True)

@router.get("/", response_model=NavigateResponse)
async def get_identify() -> responses.JSONResponse:
    # IdentifyBase.send_cmd_vel()
    return responses.JSONResponse (
        { 'data':  await NavigateBase.launch_client() }, 
        status_code=status.HTTP_200_OK
    )

@router.get("/id/{robot_id}")
async def get_identify_id(robot_id: int) -> responses.JSONResponse:
    result = await NavigateBase.launch_client(robot_id)
    if(not result):
        return responses.JSONResponse ({ 'data':  "Request to identify service failed !" } , status_code=status.HTTP_404_NOT_FOUND)
    else:
        return responses.JSONResponse ({ 'data':  result }, status_code=status.HTTP_200_OK)