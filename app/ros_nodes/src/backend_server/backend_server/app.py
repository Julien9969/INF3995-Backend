import asyncio
import time, rclpy
from fastapi.middleware.cors import CORSMiddleware
from contextlib import asynccontextmanager
from fastapi import FastAPI, Request, Response, HTTPException
from fastapi.responses import JSONResponse
from starlette.status import HTTP_504_GATEWAY_TIMEOUT

from backend_server.db.models.tables_models import Base, populate_db
from backend_server.db.utils import check_db_connected, check_db_disconnected
from backend_server.db.session import engine
from backend_server.api.base import api_router

from backend_server.websocket.base import socket_app

@asynccontextmanager
async def app_lifespan(app: FastAPI):
    # Start up event
    if(not rclpy.ok()):
        rclpy.init()
    print("Starting up")
    rclpy.init()
    await check_db_connected()
    print(app.title)

    yield

    if rclpy.ok():
        rclpy.shutdown()
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
    populate_db()
    return app

app = start_application()

@app.middleware("http")
async def timeout_middleware(request: Request, call_next):
    try:
        start_time = time.time()
        return await asyncio.wait_for(call_next(request), timeout=20)

    except asyncio.TimeoutError:
        process_time = time.time() - start_time
        return JSONResponse({'detail': 'Request processing time excedeed limit',
                             'processing_time': process_time},
                            status_code=HTTP_504_GATEWAY_TIMEOUT)

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

