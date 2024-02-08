from fastapi import APIRouter
from app.api.examples import routes_crud
from app.api.ping import routes_ping
from app.api.websocket import route_mission

api_router = APIRouter()
api_router.include_router(routes_crud.router, prefix="/api/something", tags=["exemple-routes"])
api_router.include_router(routes_ping.router, prefix="/api/ping", tags=["ping-routes"])
api_router.include_router(route_mission.router)