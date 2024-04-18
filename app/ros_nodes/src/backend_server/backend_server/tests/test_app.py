from unittest.mock import AsyncMock, patch, MagicMock
import fastapi
import pytest

from backend_server.app import app_lifespan


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
