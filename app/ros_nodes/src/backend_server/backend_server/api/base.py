from fastapi import APIRouter
from backend_server.api.examples import routes_crud
from backend_server.api.ping import routes_ping
from backend_server.api.identify import routes_identify
from backend_server.api.mission import routes_mission

api_router = APIRouter()
api_router.include_router(routes_crud.router, prefix="/api/something", tags=["exemple-routes"])
api_router.include_router(routes_ping.router, prefix="/api/ping", tags=["ping-routes"])
api_router.include_router(routes_identify.router, prefix="/api/identify", tags=["identify-routes"])
api_router.include_router(routes_mission.router, prefix="/api/mission", tags=["mission-routes"])
