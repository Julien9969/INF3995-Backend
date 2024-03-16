import json
import rclpy
from enum import Enum
from rclpy.node import Node
from backend_server.schemas.schemas import File, serviceError
from interfaces.srv import FilesServer
from .files_client import FilesClientAsync

class Commands(Enum):
    FILES_TREE = "files-tree" 
    GET_FILE = "get-file"
    EDIT_FILE = "edit-file"
    UPDATE_ROBOT = "update-robot"

class ROSFilesBase:
    
    @staticmethod
    async def send_command(robot_id: int, command: Commands, content: str = "None") -> tuple[str, str]:
        
        files_client = FilesClientAsync(robot_id)

        if not hasattr(files_client, 'req'):
            files_client.destroy_node()
            
            return serviceError("Error", "Impossible de se connecter au service files !")
        
        response = await files_client.send_request(command.value, content)
        
        files_client.get_logger().info(
            f"{command.value} command response : {response.message}"
        )

        files_client.destroy_node()
        

        return response

    @staticmethod
    async def get_files_tree(robot_id: int) -> tuple[str, str]:
        response = await ROSFilesBase.send_command(robot_id, Commands.FILES_TREE)
        
        return response.message, response.content
    
    @staticmethod
    async def get_file(robot_id: int, file_name: str, file_id: int) -> tuple[str, File] | tuple[str, str]:
        
        file = json.dumps({'name': file_name, 'id': file_id})
        response = await ROSFilesBase.send_command(robot_id, Commands.GET_FILE, file)

        if response.message == 'Error':
            return response.message, response.content
        
        return response.message, File(name=file_name, id=file_id, content=response.content)
    
    @staticmethod
    async def edit_file(robot_id: int, file: File) -> tuple[str, str]:
        file = json.dumps({'name': file.name, 'id': file.id, 'content': file.content})
        response = await ROSFilesBase.send_command(robot_id, Commands.EDIT_FILE, str(file))

        if response.message == 'Error':
            return response.message, response.content

        return response.message, response.content
        
    @staticmethod
    async def update_robot(robot_id: int) -> tuple[str, str]:
        response = await ROSFilesBase.send_command(robot_id, Commands.UPDATE_ROBOT)
        
        return response.message, response.content