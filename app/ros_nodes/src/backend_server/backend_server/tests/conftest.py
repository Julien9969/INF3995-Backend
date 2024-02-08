import os
import sys
import time
from typing import Any
from typing import Generator

import pytest
from fastapi import FastAPI
from fastapi.testclient import TestClient
from sqlalchemy import create_engine
from sqlalchemy.orm import Session
from sqlalchemy.orm import sessionmaker

# Give Access the app. module
sys.path.append(".")

from db.session import Base, get_db
from api.base import api_router


def start_application():
    app = FastAPI()
    app.include_router(api_router)
    return app


# Will see if we use the same database for testing (if yes create a clean routine after test)
while True:
    try:
        SQLALCHEMY_DATABASE_URL = "postgresql://test_eq102:test_root@test_db:5555/test_inf3995"
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
    Base.metadata.create_all(engine)  # Create the tables.
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
