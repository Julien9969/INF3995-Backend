from unittest.mock import AsyncMock, patch

import pytest
from backend_server.classes.common import WebsocketsEvents, MissionStatus
from backend_server.models.mission import Mission


@pytest.fixture
def mock_sio(mocker):
    mock = mocker.patch('backend_server.websocket.base.sio', autospec=True)
    mock.emit = AsyncMock()
    return mock


@pytest.fixture
def mock_log_manager(mocker):
    mock = mocker.patch('backend_server.ros.managers.logs.LogManager', autospec=True)
    mock.start_record_logs = AsyncMock()
    mock.stop_record_logs = AsyncMock()
    return mock


@pytest.fixture
def mock_map_manager(mocker):
    mock = mocker.patch('backend_server.ros.managers.map.MapManager', autospec=True)
    mock.start_map_listener = AsyncMock()
    mock.stop_map_listener = AsyncMock()
    return mock


@pytest.mark.asyncio
async def test_get_mission_status(mock_sio):
    mission = Mission()
    await mission.get_mission_status('sid')
    mock_sio.emit.assert_awaited_once_with(WebsocketsEvents.MISSION_STATUS.value, MissionStatus(), to='sid')


@pytest.mark.asyncio
async def test_set_mission_start(mock_sio, mock_log_manager):
    with patch('backend_server.models.mission.Mission', return_value='success'):
        mission = Mission()
        await mission.set_mission_start('sid')
    mock_sio.emit.assert_awaited_with(WebsocketsEvents.MISSION_START.value)
    mock_log_manager.start_record_logs.assert_awaited_once()


@pytest.mark.asyncio
async def test_start_record_map(mock_sio, mock_map_manager):
    mission = Mission()
    await mission.start_record_map('sid')
    mock_sio.emit.assert_awaited_with(WebsocketsEvents.MISSION_MAP.value)
    mock_map_manager.start_map_listener.assert_awaited_once()


@pytest.mark.asyncio
async def test_set_mission_end(mock_sio, mock_log_manager, mock_map_manager):
    with patch('backend_server.models.mission.Mission', return_value='success'):
        mission = Mission()
        await mission.set_mission_end('sid')
    mock_sio.emit.assert_awaited_with(WebsocketsEvents.MISSION_END.value)
    mock_log_manager.stop_record_logs.assert_awaited_once()
    mock_map_manager.stop_map_listener.assert_awaited_once()
