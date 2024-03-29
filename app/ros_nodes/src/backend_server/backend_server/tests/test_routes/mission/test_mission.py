import json
import pytest
from backend_server.websocket.events import Events
from backend_server.websocket.event_handlers.mission import get_mission_status
from unittest.mock import patch, MagicMock

def side_effect(type,json=None,to=None):
    pass


#                                             TESTS FOR get_mission_status
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


#                                             TESTS FOR set_mission_start


#                                             TESTS FOR set_mission_end



