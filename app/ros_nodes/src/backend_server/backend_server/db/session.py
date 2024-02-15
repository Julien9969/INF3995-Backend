import time
from typing import Generator

from sqlalchemy import create_engine
from sqlalchemy.orm import declarative_base
from sqlalchemy.orm import sessionmaker


while True:
	try:
		SQLALCHEMY_DATABASE_URL = "postgresql://eq102:root@host.docker.internal:5432/inf3995"
		Base = declarative_base()

		engine = create_engine(
			SQLALCHEMY_DATABASE_URL
		)
		SessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=engine)
		time.sleep(1)
		break
	except Exception as e:
		print("Waiting for db to be ready...")
		continue
     
def get_db() -> Generator:
    try:
        db = SessionLocal()
        yield db
    finally:
        db.close()
