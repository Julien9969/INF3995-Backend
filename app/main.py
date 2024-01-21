from fastapi import FastAPI, Depends, HTTPException, status
from fastapi.middleware.cors import CORSMiddleware
from contextlib import asynccontextmanager
from fastapi.responses import JSONResponse
from sqlalchemy.orm import Session

from . import crud, models, schemas
from .database import SessionLocal, engine
from typing import List


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


@app.get("/api/{id}", tags=["Something"], response_model=List[schemas.SomethingBase])
async def something(id: int, db: Session = Depends(get_db)):
    return [crud.get_something(db=db, id=id)]

@app.post("/api/something", tags=["postText"])
async def postSomething(something: str, db: Session = Depends(get_db)):

    if crud.post_string(something=something, db=db):
        return JSONResponse(content={"message": "Resource created successfully"})
    else:
        raise HTTPException(status_code=status.HTTP_500_INTERNAL_SERVER_ERROR, detail="Failed to create resource")