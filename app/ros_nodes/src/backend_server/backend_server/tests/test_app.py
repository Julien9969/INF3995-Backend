from unittest.mock import AsyncMock, patch, MagicMock
import fastapi
import pytest
from backend_server.app import start_application, create_tables, include_router, app_lifespan


# Cant get the fastapi mock for the moment
@patch("backend_server.app.create_tables")
@patch("backend_server.app.include_router")
@patch("fastapi.FastAPI.__init__", new=MagicMock(return_value=None))
def test_start_application(include_router_mock, create_tables_mock):
    app = start_application()

    assert include_router_mock.called
    assert create_tables_mock.called


@pytest.mark.asyncio
@patch("backend_server.app.check_db_disconnected", new_callable=AsyncMock)
@patch("backend_server.app.check_db_connected", new_callable=AsyncMock)
async def test_app_lifespan(check_db_connected_mock: MagicMock, check_db_disconnected_mock: MagicMock):
    app_mock = MagicMock(spec=["title"])
    app_mock.title = "TestApp"
    
    async with app_lifespan(app_mock):
        check_db_connected_mock.assert_called_once()
        # should stop at yield
        check_db_disconnected_mock.assert_not_called()
