from fastapi import APIRouter, status
from fastapi import responses, status
from .ping import PingBase, PingResponse
import asyncio

router = APIRouter(include_in_schema=True)

@router.get("/", response_model=PingResponse)
async def get_ping() -> responses.JSONResponse:
    return responses.JSONResponse (
        { 'data':  "test" }, 
        status_code=status.HTTP_200_OK
    )

@router.get("/robot", response_model=PingResponse)
async def get_ping_robot(x: float = 0.0, z: float = 0.0, robot: bool = False) -> responses.JSONResponse:
    
    # asyncio.create_task(PingBase.send_cmd_vel(x, z, robot))
    await PingBase.send_cmd_vel(x, z, robot)
    
    return responses.JSONResponse (
        { 'data':  PingBase.ping() }, 
        status_code=status.HTTP_200_OK
    )