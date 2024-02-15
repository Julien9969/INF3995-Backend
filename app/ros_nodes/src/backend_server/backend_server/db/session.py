import time
from typing import Generator

from sqlalchemy import create_engine
from sqlalchemy.orm import declarative_base
from sqlalchemy.orm import sessionmaker
import socket

def get_container_ip(container_name):
    try:
        # Resolve the container name to its IP address
        ip_address = socket.gethostbyname(container_name)
        return ip_address
    except socket.gaierror:
        print(f"Error: Unable to resolve IP address for container '{container_name}'.")
        return None


while True:
	try:
		database_container_name = 'inf3995-backend-db-1'
		database_ip = get_container_ip(database_container_name)

		SQLALCHEMY_DATABASE_URL = "postgresql://eq102:root@" + database_ip + ":5432/inf3995"
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
