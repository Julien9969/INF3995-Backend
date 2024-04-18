from backend_server.api.files import routes_files
from backend_server.api.history import routes_mission_history
from backend_server.api.identify import routes_identify
from fastapi import APIRouter

api_router = APIRouter()
api_router.include_router(routes_identify.router, prefix="/api/identify", tags=["identify-routes"])
api_router.include_router(routes_files.router, prefix="/api/files", tags=["files-routes"])
api_router.include_router(routes_mission_history.router, prefix="/api/history", tags=["history-routes"])
