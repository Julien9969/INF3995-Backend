from unittest.mock import AsyncMock, patch, MagicMock
import fastapi
from backend_server.app import start_application, create_tables, include_router, app_lifespan


# Cant get the fastapi mock for the moment
@patch("backend_server.app.create_tables")
@patch("backend_server.app.include_router")
@patch("fastapi.FastAPI.__init__", new=MagicMock(return_value=None))
def test_start_application(include_router_mock, create_tables_mock):
    app = start_application()

    assert include_router_mock.called
    assert create_tables_mock.called

