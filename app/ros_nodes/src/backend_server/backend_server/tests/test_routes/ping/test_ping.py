import asyncio, time
import unittest
from fastapi import status 
from unittest.mock import patch, MagicMock
from unittest import mock
import pytest

# mock for all file
@pytest.fixture(scope='session', autouse=True)
def sleep_mock():
    with mock.patch.object(time, 'sleep') as _fixture:
        yield _fixture

@patch("backend_server.api.ping.ping.PingBase.ping")
def test_get_ping(ping_mock, client):
    ping_mock.return_value = "mocked pong!"

    response = client.get("/api/ping/")
    assert response.status_code == 200
    assert ping_mock.called
    assert str(response.json()["data"]).find("mocked pong!") != -1

@patch("backend_server.api.ping.ping.PingBase.ping")
@patch("backend_server.api.ping.ping.PingBase.send_cmd_vel")
def test_get_ping_robot(cmd_vel_mock: MagicMock, ping_mock: MagicMock, client):
    ping_mock.return_value = "mocked pong!"
    cmd_vel_mock.return_value = asyncio.sleep(0)

    response = client.get("/api/ping/robot/")
    assert response.status_code == 200
    assert cmd_vel_mock.calledwith(0.0, 0.0, False)
    assert str(response.json()["data"]).find("mocked pong!") != -1


from backend_server.api.ping.ping import PingBase
import rclpy

@patch("rclpy.create_node")
def test_send_cmd_vel(mock_create_node: MagicMock):
    mock_node = MagicMock()
    mock_publisher = MagicMock()
    mock_publisher.publish = MagicMock()
    mock_create_node.return_value = mock_node
    mock_node.create_publisher.return_value = mock_publisher

    asyncio.run(PingBase.send_cmd_vel(1.0, 2.0, True))

    mock_create_node.assert_called_once_with('twist_publisher')
    mock_node.create_publisher.assert_called_once()
    assert mock_publisher.publish.call_count == 20
    assert time.sleep.call_count == 20

@patch("datetime.datetime")
def test_ping(mock_datetime):
    mock_datetime.now.return_value.strftime.return_value = "mocked date"
    assert PingBase.ping() == "mocked date - pong!"
    mock_datetime.now.return_value.strftime.assert_called_once_with('%d/%m/%Y, %H:%M:%S')