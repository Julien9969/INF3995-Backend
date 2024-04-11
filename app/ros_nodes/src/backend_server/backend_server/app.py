import asyncio
import rclpy
import time
from contextlib import asynccontextmanager

from backend_server.api.base import api_router
from backend_server.db.models import Base
from backend_server.db.session import engine
from backend_server.db.utils import check_db_connected, check_db_disconnected
from backend_server.db.insertions import init_missions
from backend_server.websocket.base import socket_app
from fastapi import FastAPI, Request
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse
from starlette.status import HTTP_504_GATEWAY_TIMEOUT


@asynccontextmanager
async def app_lifespan(app: FastAPI):
    # Start up event
    if not rclpy.ok():
        rclpy.init()
    await check_db_connected()

    yield

    if rclpy.ok():
        rclpy.shutdown()
    # Shutdown event
    await check_db_disconnected()


def start_application() -> FastAPI:
    app = FastAPI(lifespan=app_lifespan, debug=True, title="Limousine Backend Server 3995", version="1.0")
    app.include_router(api_router)
    # configure_static(app)
    app.mount("/", socket_app)  # Add web sockets to app
    Base.metadata.create_all(bind=engine)
    init_missions()
    return app


app = start_application()


@app.middleware("http")
async def timeout_middleware(request: Request, call_next):
    try:
        start_time = time.time()
        return await asyncio.wait_for(call_next(request), timeout=5)

    except asyncio.TimeoutError:
        process_time = time.time() - start_time
        return JSONResponse({'detail': 'Request processing time excedeed limit',
                             'processing_time': process_time},
                            status_code=HTTP_504_GATEWAY_TIMEOUT)


origins = [
    "*"
]

app.add_middleware(
    CORSMiddleware,
    allow_origins=origins,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)
