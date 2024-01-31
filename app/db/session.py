import time
from typing import Generator

from sqlalchemy import create_engine
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.orm import sessionmaker


while True:
	try:
		SQLALCHEMY_DATABASE_URL = "postgresql://eq102:root@db/inf3995"
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
     


# if you don't want to install postgres or any database, use sqlite, a file system based database,
# uncomment below lines if you would like to use sqlite and comment above 2 lines of SQLALCHEMY_DATABASE_URL AND engine

# SQLALCHEMY_DATABASE_URL = "sqlite:///./sql_app.db"
# engine = create_engine(
#     SQLALCHEMY_DATABASE_URL, connect_args={"check_same_thread": False}
# )

def get_db() -> Generator:
    try:
        db = SessionLocal()
        yield db
    finally:
        db.close()
