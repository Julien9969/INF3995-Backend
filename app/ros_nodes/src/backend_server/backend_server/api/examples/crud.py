from fastapi import HTTPException
from sqlalchemy.orm import Session
from fastapi import Request
from backend_server.db.models.tables_models import Something, stringTable

class SomethingBase:
    def __init__(self, request: Request, id: int, name: str):
        self.request: Request = request
        self.id = id
        self.name = name

    def get_something(self, db: Session, id: int):
        db_something = db.query(Something).filter_by(id=id).first()
        if db_something == None:
            raise HTTPException(status_code=404, detail="Something not found")
        return db_something


    def post_string(self, db: Session, something: str):   
        db_something = stringTable(txt=something)
        db.add(db_something)
        db.commit()
        db.refresh(db_something)
        return db_something