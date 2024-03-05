from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from contextlib import asynccontextmanager

from backend_server.db.models.exemples_models import Base
from backend_server.db.utils import check_db_connected, check_db_disconnected
from backend_server.db.session import engine
from backend_server.api.base import api_router

from backend_server.websocket.base import socket_app


@asynccontextmanager
async def app_lifespan(app: FastAPI):
    # Start up event
    print("Starting up")
    await check_db_connected()
    print(app.title)

    yield

    # Shutdown event
    await check_db_disconnected()


def include_router(app):
    app.include_router(api_router)


def create_tables():
    Base.metadata.create_all(bind=engine)


def start_application() -> FastAPI:
    app = FastAPI(lifespan=app_lifespan, debug=True, title="API", version="0.1")
    include_router(app)
    # configure_static(app)
    app.mount("/", socket_app)  # Add web sockets to app
    create_tables()
    return app


# ENLEVE TEMPORAIREMENT POUR ETRE LANCE PAR ROS A LA PLACE
app = start_application()
origins = [
    # "http://localhost",
    # "http://localhost:8000",
    # "http://127.0.0.1:8000",
    "*"
]

app.add_middleware(
    CORSMiddleware,
    allow_origins=origins,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

