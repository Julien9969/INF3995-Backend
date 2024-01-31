from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from contextlib import asynccontextmanager

from app.db.models.exemples_models import Base
from app.db.utils import check_db_connected, check_db_disconnected
from app.db.session import engine
from app.router.base import api_router

# Don't work for the moment, on_event work of you want
@asynccontextmanager
async def app_lifespan(app: FastAPI):
    # Start up event
    print("Starting up")
    await check_db_connected()
    
    yield
    
    # Shutdown event
    await check_db_disconnected()

def include_router(app):
    app.include_router(api_router)

# def configure_static(app):
#     app.mount("/static", StaticFiles(directory="static"), name="static")

def create_tables():
    Base.metadata.create_all(bind=engine)

def start_application() -> FastAPI:
    app = FastAPI(title="INF3995", version="V1.0.0", lifespan=app_lifespan)
    include_router(app)
    # configure_static(app)
    create_tables()
    return app

app = start_application()

origins = [
    "http://localhost",
    "http://localhost:8000",
    "http://127.0.0.1:8000",
    "*"
]

app.add_middleware(
    CORSMiddleware,
    allow_origins=origins,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# @app.on_event("startup")
# async def startup_event():
#     async with lifespan(app):
#         print("App started")
# 
# @app.on_event("shutdown")
# async def shutdown_event():
#     print("App shutting down")
#     await check_db_disconnected()


# @app.on_event("startup")
# async def app_startup():
#     await check_db_connected()
# 
# 
# @app.on_event("shutdown")
# async def app_shutdown():
#     await check_db_disconnected()
