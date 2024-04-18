import os
import sys
import time
from typing import Any
from typing import Generator
from unittest.mock import patch

import pytest
from fastapi import FastAPI
from fastapi.testclient import TestClient
from sqlalchemy import create_engine
from sqlalchemy.orm import Session
from sqlalchemy.orm import sessionmaker

sys.path.append(".")
# print(sys.path)
# print(os.getcwd())

# ROS2 Mock (make ros2 installation not required for testing)
import backend_server.tests.mock.rclpy_mock as rclpy_mock

patch.dict("sys.modules", rclpy=rclpy_mock).start()

import backend_server.tests.mock.geometry_msgs_mock as geometry_msgs_mock

patch.dict("sys.modules", geometry_msgs=geometry_msgs_mock).start()

import backend_server.tests.mock.rcl_interfaces_mock as rcl_interfaces_mock

patch.dict("sys.modules", rcl_interfaces=rcl_interfaces_mock).start()

import backend_server.tests.mock.interfaces_mock as interfaces_mock

patch.dict("sys.modules", interfaces=interfaces_mock).start()

import backend_server.tests.mock.nav_msgs as nav_msgs_msg_mock

patch.dict("sys.modules", nav_msgs=nav_msgs_msg_mock).start()

from backend_server.api.base import api_router


def start_application():
    app = FastAPI()
    app.include_router(api_router)
    return app


# Will see if we use the same database for testing (if yes create a clean routine after test)
from backend_server.db.session import Base, get_db

while True:
    try:
        # SQLALCHEMY_DATABASE_URL = "postgresql://test_eq102:test_root@test_db:5555/test_inf3995"
        # Temporary use the same database for testing as no test realy use it
        environment = os.getenv("SQLALCHEMY_DATABASE_HOST", "test_db")

        if environment == "test_db":
            SQLALCHEMY_DATABASE_URL = "postgresql://eq102:root@test_db:5430/inf3995"
        else:
            SQLALCHEMY_DATABASE_URL = f"postgresql://eq102:root@{environment}:5430/inf3995"

        print(f"Using database: {SQLALCHEMY_DATABASE_URL}")
        engine = create_engine(
            SQLALCHEMY_DATABASE_URL
        )

        SessionTesting = sessionmaker(autocommit=False, autoflush=False, bind=engine)
        break
    except Exception as e:
        print("Waiting for db to be ready...")
        time.sleep(1)


@pytest.fixture(scope="module")
def app() -> Generator[FastAPI, Any, None]:
    """
    Create a fresh database on each test case.
    """
    Base.metadata.create_all(engine)
    _app = start_application()
    yield _app
    Base.metadata.drop_all(engine)


@pytest.fixture(scope="module")
def db_session(app: FastAPI) -> Generator[SessionTesting, Any, None]:
    connection = engine.connect()
    transaction = connection.begin()
    session = SessionTesting(bind=connection)
    if session is None:
        print("Test Database session is None.")
    else:
        print("Test Database session is On.")
    yield session  # use the session in tests.
    session.close()
    transaction.rollback()
    connection.close()


@pytest.fixture(scope="module")
def client(
        app: FastAPI, db_session: SessionTesting
) -> Generator[TestClient, Any, None]:
    """
    Create a new FastAPI TestClient that uses the `db_session` fixture to override
    the `get_db` dependency that is injected into routes.
    """

    def _get_test_db():
        try:
            yield db_session
        finally:
            pass

    app.dependency_overrides[get_db] = _get_test_db
    with TestClient(app) as client:
        yield client


@pytest.fixture(scope="module")
def normal_user_token_headers(client: TestClient, db_session: Session):
    return None
