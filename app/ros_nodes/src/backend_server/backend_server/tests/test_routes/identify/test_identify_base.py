from unittest.mock import patch, MagicMock, AsyncMock

import pytest
from backend_server.api.identify.identify_base import IdentifyBase

@pytest.mark.asyncio
@patch("backend_server.api.identify.identify_base.IdentifyClientAsync")
async def test_launch_client(mock_identify_client):
    mock_identify_client.return_value.send_request = AsyncMock()
    mock_identify_client.return_value.send_request.return_value = MagicMock(b=8)
    response = await IdentifyBase.launch_client(4)
    assert response == 'Result of identify: for 4 * 2 = 8'
    mock_identify_client.assert_called_once_with(4)
    mock_identify_client.return_value.send_request.assert_called_once_with(4)
    mock_identify_client.return_value.destroy_node.assert_called_once()


@pytest.mark.asyncio
@patch("backend_server.api.identify.identify_base.hasattr")
@patch("backend_server.api.identify.identify_base.IdentifyClientAsync", spec=None)
async def test_launch_client_failure(mock_identify_client, mock_hasattr): 
    mock_hasattr.return_value = False
    mock_identify_client.return_value.send_request = AsyncMock()
    mock_identify_client.return_value.send_request.return_value = None
    response = await IdentifyBase.launch_client(4)
    assert response is None
    mock_identify_client.return_value.destroy_node.assert_called_once()

