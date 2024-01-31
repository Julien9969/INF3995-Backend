from fastapi import APIRouter
from app.router.examples import routes_crud


api_router = APIRouter()
api_router.include_router(routes_crud.router, prefix="", tags=["exemple-routes"])
