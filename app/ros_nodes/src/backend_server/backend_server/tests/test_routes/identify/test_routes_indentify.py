import pytest
from fastapi import status 
from unittest.mock import patch, MagicMock, AsyncMock
from backend_server.api.identify.identify_base import IdentifyBase

@patch.object(IdentifyBase, "launch_client", return_value="launch_client")
def test_get_identify(launch_client_mock, client):
    response = client.get("/api/identify/")
    assert response.status_code == status.HTTP_200_OK
    assert launch_client_mock.called
    assert str(response.json()["data"]) == "launch_client"

@pytest.mark.asyncio
@patch.object(IdentifyBase, "launch_client", return_value="launch_client")
async def test_get_identify_id(launch_client_mock, client):
    robot_id = 1
    response = client.get(f"/api/identify/id/{robot_id}")
    assert response.status_code == status.HTTP_200_OK
    assert launch_client_mock.calledwith(robot_id)
    assert str(response.json()["data"]) == "launch_client"

@pytest.mark.asyncio
@patch.object(IdentifyBase, "launch_client", return_value=False)
async def test_get_identify_id_failure(launch_client_mock, client):
    robot_id = 1
    response = client.get(f"/api/identify/id/{robot_id}")
    assert response.status_code == status.HTTP_404_NOT_FOUND
    assert launch_client_mock.calledwith(robot_id)
    assert str(response.json()["data"]) == "Request to identify service failed !"

