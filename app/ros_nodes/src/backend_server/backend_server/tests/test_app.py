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

@pytest.fixture
async def app():
    return fastapi.FastAPI(title="TestApp")

@pytest.mark.asyncio
async def test_app_lifespan():
    async with app_lifespan(await app()) as ctx:
        # Here you can write your test logic
        
        # Mock any necessary async calls for testing purposes
        ctx.check_db_connected = AsyncMock(return_value=None)
        ctx.check_db_disconnected = AsyncMock(return_value=None)
        
        # Assert any expected behavior or state changes
        
        # Example assertion, you can replace this with your actual test logic
        assert ctx.check_db_connected.called
        assert ctx.check_db_disconnected.called