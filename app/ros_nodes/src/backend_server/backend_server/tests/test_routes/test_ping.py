import unittest
from fastapi import status 
from unittest.mock import patch, MagicMock
from unittest import mock
import pytest

    
# @patch("backend_server.api.ping.ping.PingBase")
# def test_ping(client, ping_mock):
#     ping_mock.return_value.ping.return_value = "mocked pong!"

#     response = client.get("/api/ping/")
#     assert response.status_code == 200
#     assert ping_mock.called
#     assert str(response.json()["data"]).find("pong!") != -1

@pytest.fixture
def ping_mock():
    with patch("backend_server.api.ping.ping.PingBase") as mock:
        yield mock

def test_ping(client, ping_mock):
    ping_mock.return_value.ping.return_value = "mocked pong!"

    response = client.get("/api/ping/")
    assert response.status_code == 200
    assert ping_mock.called
    assert str(response.json()["data"]).find("pong!") != -1