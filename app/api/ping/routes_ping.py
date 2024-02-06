from fastapi import APIRouter, status
from fastapi import responses, status
from .ping import PingBase, PingResponse

router = APIRouter(include_in_schema=True)

@router.get("/", response_model=PingResponse)
async def get_ping() -> responses.JSONResponse:
    return responses.JSONResponse (
        { 'data':  PingBase.ping() }, 
        status_code=status.HTTP_200_OK
    )