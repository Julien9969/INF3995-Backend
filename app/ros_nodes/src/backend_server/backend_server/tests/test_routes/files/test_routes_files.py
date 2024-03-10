import pytest
from httpx import AsyncClient
from fastapi import status
from unittest.mock import patch, MagicMock, AsyncMock
from backend_server.schemas.schemas import FileObject, File
from backend_server.api.files.files_base import ROSFilesBase

@pytest.mark.asyncio
@patch.object(ROSFilesBase, "get_files_tree", return_value=("Success", {"data": "tree"}))
async def test_get_files_tree_success(get_files_tree_mock, client: AsyncClient):
    response = client.get("/api/files/tree/1")
    assert response.status_code == status.HTTP_200_OK
    assert response.json() == {"data": "tree"}
    get_files_tree_mock.assert_called_once_with(1)

@pytest.mark.asyncio
@patch.object(ROSFilesBase, "get_files_tree", return_value=("Error", None))
async def test_get_files_tree_failure(get_files_tree_mock, client: AsyncClient):
    response = client.get("/api/files/tree/1")
    assert response.status_code == status.HTTP_404_NOT_FOUND
    assert response.json() == "Error"
    get_files_tree_mock.assert_called_once_with(1)

@pytest.mark.asyncio
@patch.object(ROSFilesBase, "get_file", return_value=("Success", FileObject("file", 1, "content")))
async def test_get_file_success(get_file_mock, client: AsyncClient):
    response = client.get("/api/files/file?robot_id=1&name=file&id=1")
    assert response.status_code == status.HTTP_200_OK
    assert response.json() == {"name": "file", "id": 1, "content": "content"}
    get_file_mock.assert_called_once_with(1, "file", 1)

@pytest.mark.asyncio
@patch.object(ROSFilesBase, "get_file", return_value=("Error", "File not found"))
async def test_get_file_failure(get_file_mock, client: AsyncClient):
    response = client.get("/api/files/file?robot_id=1&name=file&id=1")
    assert response.status_code == status.HTTP_404_NOT_FOUND
    assert response.json() == "File not found"
    get_file_mock.assert_called_once_with(1, "file", 1)

@pytest.mark.asyncio
@patch.object(ROSFilesBase, "edit_file", return_value=("Success", None))
async def test_post_file_success(edit_file_mock, client: AsyncClient):
    file_data = File(name="file", id=1, content="content")
    response = client.post("/api/files/save/1", json=file_data.model_dump())
    assert response.status_code == status.HTTP_200_OK
    assert response.json() == "File saved"
    edit_file_mock.assert_called_once_with(1, file_data)

@pytest.mark.asyncio
@patch.object(ROSFilesBase, "edit_file", return_value=("Error", "Failed to save file"))
async def test_post_file_failure(edit_file_mock, client: AsyncClient):
    file_data = File(name="file", id=1, content="content")
    response = client.post("/api/files/save/1", json=file_data.model_dump())
    assert response.status_code == status.HTTP_404_NOT_FOUND
    assert response.json() == "Failed to save file"
    edit_file_mock.assert_called_once_with(1, file_data)
# 
@pytest.mark.asyncio
@patch.object(ROSFilesBase, "update_robot", return_value=("Success", "Robot updated"))
async def test_patch_update_robot_success(update_robot_mock, client: AsyncClient):
    response = client.patch("/api/files/update/1")
    assert response.status_code == status.HTTP_200_OK
    assert response.json() == "Robot updated"
    update_robot_mock.assert_called_once_with(1)

@pytest.mark.asyncio
@patch.object(ROSFilesBase, "update_robot", return_value=("Error", "Failed to update robot"))
async def test_patch_update_robot_failure(update_robot_mock, client: AsyncClient):
    response = client.patch("/api/files/update/1")
    assert response.status_code == status.HTTP_404_NOT_FOUND
    assert response.json() == "Failed to update robot"
    update_robot_mock.assert_called_once_with(1)

@pytest.mark.asyncio
async def test_test(client: AsyncClient):
    response = client.get("/api/files/test")
    assert response.status_code == status.HTTP_200_OK
    assert response.json() == {"data": "test"}
