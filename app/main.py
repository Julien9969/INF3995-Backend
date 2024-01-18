from fastapi import FastAPI, Depends
from fastapi.middleware.cors import CORSMiddleware
from contextlib import asynccontextmanager
from sqlalchemy.orm import Session

from . import crud, models, schemas
from .database import SessionLocal, engine

models.Base.metadata.create_all(bind=engine)

# Dependency
def get_db():
    db = SessionLocal()
    try:
        yield db
    finally:
        db.close()

        
@asynccontextmanager
async def lifespan(app: FastAPI): 
    yield 

app = FastAPI(lifespan=lifespan)

origins = [
    "http://localhost",
    "http://localhost:4200",
    "http://127.0.0.1:4200",
    "*"
]

app.add_middleware(
    CORSMiddleware,
    allow_origins=origins,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


@app.get("/api/{id}", tags=["Something"], response_model=list[schemas.SomethingBase])
async def something(id: int, db: Session = Depends(get_db)):
    return [crud.get_something(db=db, id=id)]
