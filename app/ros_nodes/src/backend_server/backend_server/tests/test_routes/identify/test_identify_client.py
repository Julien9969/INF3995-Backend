from unittest.mock import AsyncMock, patch, MagicMock

import pytest
from backend_server.api.identify.identify_client import IdentifyClientAsync

@patch("interfaces.srv.Identify")
@patch("rclpy.node.Node.create_client")
def test_identify_client_init(create_client_mock, identify_mock):
    identify_mock.Request.return_value = MagicMock()
    cli_mock = MagicMock()
    cli_mock.wait_for_service.return_value = True
    create_client_mock.return_value = cli_mock
    identify_client = IdentifyClientAsync(1)
    assert create_client_mock.called
    assert hasattr(identify_client, 'req')
    identify_client.cli.wait_for_service.assert_called_once_with(timeout_sec=5.0)


@patch("rclpy.node.Node.get_logger")
@patch("interfaces.srv.Identify")
@patch("rclpy.node.Node.create_client")
def test_identify_client_init_failure(create_client_mock, identify_mock, get_logger_mock):
    identify_mock.Request.return_value = MagicMock()
    cli_mock = MagicMock()
    cli_mock.wait_for_service.return_value = False
    create_client_mock.return_value = cli_mock
    get_logger_mock.return_value = MagicMock()

    identify_client = IdentifyClientAsync(1)
    assert create_client_mock.called
    assert not hasattr(identify_client, 'req')
    identify_client.get_logger().info.assert_called_once_with('service not available (robot id 1)...')


@pytest.mark.asyncio
@patch("rclpy.node.Node.create_client")
async def test_send_request(create_client_mock):
    future = MagicMock()
    future.result = MagicMock(return_value="result")
    cli_mock = MagicMock()
    cli_mock.call_async.return_value = future
    create_client_mock.return_value = cli_mock

    identify_client = IdentifyClientAsync(1)

    result = await identify_client.send_request("a")
    assert result == "result"