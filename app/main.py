from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from contextlib import asynccontextmanager

from app.db.models.exemples_models import Base
from app.db.utils import check_db_connected, check_db_disconnected
from app.db.session import engine
from app.api.base import api_router

from .websocket.websocket import sio
import socketio
import logging

logging.basicConfig(level=logging.DEBUG,
                    format='%(asctime)s - %(levelname)s - %(message)s',
                    datefmt='%Y-%m-%d %H:%M:%S')

origins = [
    "http://localhost:4200",
    "*",
]


# Don't work for the moment, on_event work of you want
@asynccontextmanager
async def app_lifespan(app: FastAPI):
    await check_db_connected()
    yield  # This creates a context manager for the database
    #        that will last the entire lifespan of the application
    await check_db_disconnected()


def start_application() -> FastAPI:
    app = FastAPI(
        lifespan=app_lifespan,
        debug=True,
        title="API",
        version="0.1"
    )
    socket_app = socketio.ASGIApp(sio, app)
    app.mount("/", socket_app)
    app.include_router(api_router)
    Base.metadata.create_all(bind=engine)
    return app


app = start_application()
app.add_middleware(
    CORSMiddleware,
    allow_origins=origins,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# @app.on_event("startup")
# async def app_startup():
#     await check_db_connected()
# 
# 
# @app.on_event("shutdown")
# async def app_shutdown():
#     await check_db_disconnected()
