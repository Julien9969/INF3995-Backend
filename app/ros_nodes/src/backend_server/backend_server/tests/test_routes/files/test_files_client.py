import asyncio
import sys
from interfaces.srv import FilesServer
import pytest
import rclpy
from rclpy.node import Node
from unittest.mock import AsyncMock, MagicMock, patch
from backend_server.api.files.files_client import FilesClientAsync


@patch("rclpy.node.Node.create_client")
def test_files_client_async_init(create_client_mock):
    ros_route = "robot1/files"
    cli_mock = MagicMock()
    create_client_mock.return_value = cli_mock
    cli_mock.wait_for_service.return_value = True

    files_client = FilesClientAsync(1)

    create_client_mock.assert_called_once_with(FilesServer, ros_route)
    assert hasattr(files_client, 'cli')
    assert hasattr(files_client, 'req')

@patch("rclpy.node.Node.get_logger")
@patch("rclpy.node.Node.create_client")
def test_files_client_async_init_failure(create_client_mock, get_logger_mock):
    ros_route = "robot1/files"
    cli_mock = MagicMock()
    create_client_mock.return_value = cli_mock
    cli_mock.wait_for_service.return_value = False
    get_logger_mock.return_value = MagicMock()

    files_client = FilesClientAsync(1)

    create_client_mock.assert_called_once_with(FilesServer, ros_route)
    assert hasattr(files_client, 'cli')
    assert not hasattr(files_client, 'req')
    files_client.get_logger().info.assert_called_once_with('service not available (robot id 1).')

@pytest.mark.asyncio
@patch("rclpy.node.Node.create_client")
async def test_files_client_async_send_request(create_client_mock):
    future_mock = AsyncMock()
    future_mock.result = MagicMock(return_value="result")
    cli_mock = MagicMock()
    cli_mock.call_async.return_value = future_mock
    create_client_mock.return_value = cli_mock

    files_client = FilesClientAsync(1)

    result = await files_client.send_request("command", "content")

    assert result == "result"
    files_client.cli.call_async.assert_called_once()
