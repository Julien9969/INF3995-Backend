from unittest.mock import patch, MagicMock, AsyncMock

import pytest
from backend_server.api.identify.identify_base import IdentifyBase
import backend_server.api.identify.identify_base as identify_base

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

@pytest.mark.asyncio
@patch("fastapi.concurrency.run_in_threadpool")
@patch("rclpy.create_node")
async def test_list_connected_robot(mock_create_node, run_in_threadpool_mock):
    mock_node = MagicMock()
    mock_subscription = MagicMock()
    
    async def custom_create_subscription(*args, **kwargs):
        print("\n\n\nCustom create subscription\n\n\n")
        return mock_subscription

    mock_node.create_subscription = MagicMock(side_effect = lambda a, b, c, d: custom_create_subscription(a, b, c, d))
    mock_create_node.return_value = mock_node
    
    mock_node.get_logger = MagicMock()
    mock_node.get_logger.return_value.info = MagicMock()
    
    response = await IdentifyBase.list_connected_robot()
    
    assert isinstance(response, list) == True
    mock_node.get_logger.assert_called()
    mock_node.destroy_node.assert_called()

@patch("backend_server.api.identify.identify_base.i", 1)
@patch("backend_server.api.identify.identify_base.connected_robots", set())
def test_odom_callback(capsys):

    IdentifyBase.odom_callback("odom_data")

    captured = capsys.readouterr()
    assert captured.out == "Received odom data from robot 1 :\n"
    assert identify_base.connected_robots == {1}
    
    identify_base.i = 2
    IdentifyBase.odom_callback("odom_data")
    captured = capsys.readouterr()
    assert captured.out == "Received odom data from robot 2 :\n"
    assert identify_base.connected_robots == {1, 2}