from unittest.mock import patch, MagicMock, AsyncMock

import pytest
from backend_server.websocket.event_handlers.mission import get_mission_status, set_mission_start, set_mission_end, \
    start_record_map
from backend_server.websocket.events import Events


def side_effect(type, json=None, to=None):
    pass


@pytest.mark.asyncio
@patch("backend_server.websocket.event_handlers.mission.sio.emit")
@patch("backend_server.websocket.event_handlers.mission.StatusUpdate", autospec=True)
async def test_get_mission_status(mock_StatusUpdate, mock_sio_emit):
    mock_sio_emit.side_effect = side_effect
    mock_status = MagicMock()
    mock_status.to_json.return_value = "empty"
    mock_StatusUpdate.return_value = mock_status
    await get_mission_status(1)
    mock_StatusUpdate.assert_called_once()
    mock_sio_emit.assert_called_once_with(Events.MISSION_STATUS.value, "empty", to=1)


@pytest.mark.asyncio
@patch("backend_server.websocket.logs", autospec=True)
@patch("backend_server.websocket.event_handlers.mission.sio.emit", autospec=True)
@patch("backend_server.api.mission", autospec=True)
async def test_set_mission_start(mock_mission, mock_sio_emit, mock_logs):
    mock_sio_emit.side_effect = side_effect
    mock_logs = MagicMock()
    mock_mission = MagicMock()
    mock_logs.send_log.return_value = MagicMock()
    mock_mission.start_mission.return_value = "Mission started"
    mock_mission.error.return_value = "Mission not started"
    await set_mission_start(1)
    mock_mission.assert_called_once()
    mock_sio_emit.assert_called_once_with(Events.MISSION_START.value, "empty", to=1)


@pytest.mark.asyncio
async def test_start_record_map():
    with patch("backend_server.websocket.event_handlers.mission.sio.emit", autospec=True) as mock_sio_emit, \
         patch("backend_server.websocket.event_handlers.map.MapManager", autospec=True) as mock_MapManager, \
         patch("backend_server.websocket.logs.send_log", autospec=True) as mock_send_log:
        mock_sio_emit.side_effect = side_effect
        mock_send_log.return_value = AsyncMock()
        mock_MapManager.start_map_listener.return_value = AsyncMock()
        await start_record_map(1)
        mock_send_log.assert_called_once()
        mock_sio_emit.assert_called_once_with(Events.MISSION_MAP.value, "empty", to=1)


@pytest.mark.asyncio
@patch("backend_server.websocket.event_handlers.mission.sio.emit")
@patch("backend_server.websocket.event_handlers.mission.StatusUpdate", autospec=True)
async def test_get_mission_status(mock_StatusUpdate, mock_sio_emit):
    mock_sio_emit.side_effect = side_effect
    mock_status = MagicMock()
    mock_status.to_json.return_value = "empty"
    mock_StatusUpdate.return_value = mock_status
    await get_mission_status(1)
    mock_StatusUpdate.assert_called_once()
    mock_sio_emit.assert_called_once_with(Events.MISSION_STATUS.value,"empty" , to=1)
