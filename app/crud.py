from fastapi import HTTPException
from sqlalchemy.orm import Session

from . import models

def get_something(db: Session, id: int):
    db_something = db.query(models.Something).filter_by(id=id).first()
    if db_something == None:
        raise HTTPException(status_code=404, detail="Something not found")
    return db_something