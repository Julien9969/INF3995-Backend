import json
import pytest
from unittest.mock import patch, MagicMock, AsyncMock
from backend_server.api.files.files_base import ROSFilesBase, Commands
from backend_server.schemas.schemas import File, serviceError

class Response:
    def __init__(self, message, content):
        self.message = message
        self.content = content

@pytest.mark.asyncio
@patch("rclpy.init")
@patch("rclpy.shutdown")
@patch("backend_server.api.files.files_base.FilesClientAsync")
async def test_send_command_success(mock_files_client, shutdown_mock, init_mock):
    mock_files_client.return_value.send_request = AsyncMock(return_value=Response("Success", "File content"))
    response = await ROSFilesBase.send_command(123, Commands.FILES_TREE)
    
    assert (response.message, response.content) == ("Success", "File content")
    mock_files_client.assert_called_once_with(123)
    mock_files_client.return_value.send_request.assert_called_once_with(
        Commands.FILES_TREE.value, "None"
    )
    mock_files_client.return_value.get_logger.assert_called_once()
    mock_files_client.return_value.destroy_node.assert_called_once()
    shutdown_mock.assert_called_once()
    init_mock.assert_called_once()

@pytest.mark.asyncio
@patch("backend_server.api.files.files_base.hasattr")
@patch("rclpy.init")
@patch("rclpy.shutdown")
@patch("backend_server.api.files.files_base.FilesClientAsync")
async def test_send_command_req_null(mock_files_client, shutdown_mock, init_mock, mock_hasattr):
    mock_hasattr.return_value = False
    mock_files_client.return_value.send_request = AsyncMock()
    
    response = await ROSFilesBase.send_command(123, Commands.FILES_TREE)
    assert (response.message, response.content) == ("Error", "Impossible de se connecter au service files !")

    mock_files_client.return_value.destroy_node.assert_called_once()
    shutdown_mock.assert_called_once()
    init_mock.assert_called_once()

@pytest.mark.asyncio
@patch("rclpy.init")
@patch("rclpy.shutdown")
@patch("backend_server.api.files.files_base.FilesClientAsync")
async def test_send_command_failure(mock_files_client, shutdown_mock, init_mock):
    mock_files_client.return_value.send_request = AsyncMock(return_value=Response("Error", "Connection failed"))
    response = await ROSFilesBase.send_command(456, Commands.FILES_TREE)
    
    assert (response.message, response.content) == ("Error", "Connection failed")
    mock_files_client.assert_called_once_with(456)
    mock_files_client.return_value.send_request.assert_called_once_with(
        Commands.FILES_TREE.value, "None"
    )
    mock_files_client.return_value.get_logger.assert_called_once()
    mock_files_client.return_value.destroy_node.assert_called_once()
    shutdown_mock.assert_called_once()
    init_mock.assert_called_once()

@pytest.mark.asyncio
@patch.object(ROSFilesBase, "send_command", return_value = Response("Success", "Tree data"))
async def test_get_files_tree_success(mock_send_command):
    
    response = await ROSFilesBase.get_files_tree(789)
    
    assert response == ("Success", "Tree data")
    mock_send_command.assert_called_once_with(789, Commands.FILES_TREE)

@pytest.mark.asyncio
@patch.object(ROSFilesBase, "send_command", return_value = Response("Error", "Failed to retrieve tree"))
async def test_get_files_tree_failure(mock_send_command):
    
    response = await ROSFilesBase.get_files_tree(789)
    
    assert response == ("Error", "Failed to retrieve tree")
    mock_send_command.assert_called_once_with(789, Commands.FILES_TREE)

@pytest.mark.asyncio
@patch.object(ROSFilesBase, "send_command", return_value = Response("Success", "file content"))
async def test_get_file_success(mock_send_command):
    
    response = await ROSFilesBase.get_file(1, "test.txt", 12)
    
    assert response == ("Success", File(name="test.txt", id=12, content="file content"))
    mock_send_command.assert_called_once_with(1, Commands.GET_FILE, '{"name": "test.txt", "id": 12}')

@pytest.mark.asyncio
@patch.object(ROSFilesBase, "send_command", return_value = Response("Error", "Failed to retrieve file"))
async def test_get_file_failure(mock_send_command):
    response = await ROSFilesBase.get_file(1, "test.txt", 122)
    
    assert response == ("Error", "Failed to retrieve file")
    mock_send_command.assert_called_once_with(1, Commands.GET_FILE, '{"name": "test.txt", "id": 122}')

@pytest.mark.asyncio
@patch.object(ROSFilesBase, "send_command", return_value = Response("Success", "File edited successfully"))
async def test_edit_file_success(mock_send_command):
    
    response = await ROSFilesBase.edit_file(789, File(name="test.txt", id=1, content="new content"))
    
    assert response == ("Success", "File edited successfully")
    mock_send_command.assert_called_once_with(789, Commands.EDIT_FILE, '{"name": "test.txt", "id": 1, "content": "new content"}')

@pytest.mark.asyncio
@patch.object(ROSFilesBase, "send_command", return_value = Response("Error", "Failed to edit file"))
async def test_edit_file_failure(mock_send_command):
    response = await ROSFilesBase.edit_file(789, File(name="test.txt", id=1, content="new content"))
    
    assert response == ("Error", "Failed to edit file")
    mock_send_command.assert_called_once_with(789, Commands.EDIT_FILE, '{"name": "test.txt", "id": 1, "content": "new content"}')

@pytest.mark.asyncio
@patch.object(ROSFilesBase, "send_command", return_value = Response("Success", "Robot updated successfully"))
async def test_update_robot_success(mock_send_command):
    
    response = await ROSFilesBase.update_robot(789)
    
    assert response == ("Success", "Robot updated successfully")
    mock_send_command.assert_called_once_with(789, Commands.UPDATE_ROBOT)

@pytest.mark.asyncio
@patch.object(ROSFilesBase, "send_command", return_value = Response("Error", "Failed to update robot"))
async def test_update_robot_failure(mock_send_command):
    
    response = await ROSFilesBase.update_robot(789)
    
    assert response == ("Error", "Failed to update robot")
    mock_send_command.assert_called_once_with(789, Commands.UPDATE_ROBOT)
