import pytest
from unittest.mock import AsyncMock, patch
from backend_server.websocket.event_handlers.mission import get_mission_status, set_mission_end, set_mission_start, start_record_map
from backend_server.websocket.base import sio
from backend_server.websocket.events import Events
from backend_server.api.mission.mission_base import start_mission, stop_mission
from backend_server.websocket.logs import send_log, LogType, Log
from backend_server.websocket.status import StatusUpdate
from backend_server.websocket.event_handlers.map import MapManager


@pytest.fixture
def mock_sio(mocker):
    mock = mocker.patch('backend_server.websocket.base.sio', autospec=True)
    mock.emit = AsyncMock()
    return mock


@pytest.fixture
def mock_log_manager(mocker):
    mock = mocker.patch('backend_server.websocket.logs.LogManager', autospec=True)
    mock.start_record_logs = AsyncMock()
    mock.stop_record_logs = AsyncMock()
    return mock


@pytest.fixture
def mock_map_manager(mocker):
    mock = mocker.patch('backend_server.websocket.event_handlers.map.MapManager', autospec=True)
    mock.start_map_listener = AsyncMock()
    mock.stop_map_listener = AsyncMock()
    return mock


# @pytest.mark.asyncio
# async def test_get_mission_status(mock_sio):
#     await get_mission_status('sid')
#     mock_sio.emit.assert_awaited_once_with(Events.MISSION_STATUS.value, StatusUpdate().to_json(), to='sid')


# @pytest.mark.asyncio
# async def test_set_mission_start(mock_sio, mock_log_manager):
#     with patch('backend_server.api.mission.mission_base.start_mission', return_value='success'):
#         await set_mission_start('sid')
#     mock_sio.emit.assert_awaited_with(Events.MISSION_START.value)
#     mock_log_manager.start_record_logs.assert_awaited_once()


# @pytest.mark.asyncio
# async def test_start_record_map(mock_sio, mock_map_manager):
#     await start_record_map('sid')
#     mock_sio.emit.assert_awaited_with(Events.MISSION_MAP.value)
#     mock_map_manager.start_map_listener.assert_awaited_once()


# @pytest.mark.asyncio
# async def test_set_mission_end(mock_sio, mock_log_manager, mock_map_manager):
#     with patch('backend_server.api.mission.mission_base.stop_mission', return_value='success'):
#         await set_mission_end('sid')
#     mock_sio.emit.assert_awaited_with(Events.MISSION_END.value)
#     mock_log_manager.stop_record_logs.assert_awaited_once()
#     mock_map_manager.stop_map_listener.assert_awaited_once()
