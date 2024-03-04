import rclpy
from rclpy.node import Node
from backend_server.schemas.schemas import File, FileId, FileObject
from interfaces.srv import FilesServer

class ROSFilesBase():
    @staticmethod
    def get_file() -> File:
        with open('./src/backend_server/backend_server/api/files/file_trees.json', 'r', encoding='utf-8') as f:
            file = FileObject(name="file1", id=1, content=f.read(), robotId=1)
            return file

    @staticmethod
    def get_files_tree(robot_id: int) -> str:
        rclpy.init()
        client = Node.create_client(FilesServer, f"robot{robot_id}/files")

        if client.wait_for_service(timeout_sec=5.0):
            client.req = FilesServer.Request()
        else:    
            Node.get_logger().info(f'service not available (robot id {robot_id}), waiting again...')

        # data = None
        # with open('./src/backend_server/backend_server/api/files/file_trees.json', 'r', encoding='utf-8') as file:
        #     data = file.read()
        # return data
    
    @staticmethod
    def post_file():
        pass