from fastapi import APIRouter
from backend_server.api.ping import routes_ping
from backend_server.api.files import routes_files
from backend_server.api.history import routes_mission_history


api_router = APIRouter()
api_router.include_router(routes_ping.router, prefix="/api/ping", tags=["ping-routes"])
api_router.include_router(routes_files.router, prefix="/api/files", tags=["files-routes"])
api_router.include_router(routes_mission_history.router, prefix="/api/history", tags=["history-routes"])

