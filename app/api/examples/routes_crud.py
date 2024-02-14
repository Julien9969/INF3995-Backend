from fastapi import APIRouter, Depends, HTTPException, status
from fastapi.responses import JSONResponse
from sqlalchemy.orm import Session
from fastapi import responses, status

from .crud import SomethingBase
from app.schemas.schemas import SomethingBase
from app.db.session import get_db
from typing import List

router = APIRouter(include_in_schema=True)

@router.get("/{id}", response_model=List[SomethingBase])
async def something(id: int, db: Session = Depends(get_db)):
    crud = SomethingBase(request=None, id=id, name=None)
    return [crud.get_something(db=db, id=id)]


@router.post("")
async def postSomething(something: str, db: Session = Depends(get_db)):
    crud = SomethingBase(request=None, id=None, name=something)
    if crud.post_string(something=something, db=db):
        return JSONResponse(content={"message": "Resource created successfully"})
    else:
        raise HTTPException(status_code=status.HTTP_500_INTERNAL_SERVER_ERROR, detail="Failed to create resource")