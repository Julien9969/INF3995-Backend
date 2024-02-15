from fastapi import APIRouter, status
from fastapi import responses, status
from .identify_base import IdentifyBase, IdentifyResponse
import asyncio


router = APIRouter(include_in_schema=True)

@router.get("/", response_model=IdentifyResponse)
async def get_identify() -> responses.JSONResponse:
    # IdentifyBase.send_cmd_vel()
    print("here")
    return responses.JSONResponse (
        { 'data':  IdentifyBase.launch_client() }, 
        status_code=status.HTTP_200_OK
    )
