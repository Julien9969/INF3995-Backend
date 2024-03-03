# import rclpy
from backend_server.schemas.schemas import File, FileId, FileObject



class ROSFilesBase():
    @staticmethod
    def get_file() -> File:
        with open('./src/backend_server/backend_server/api/files/file_trees.json', 'r', encoding='utf-8') as f:
            file = FileObject(name="file1", id=1, content=f.read())
            return file

    @staticmethod
    def get_files_tree():
        data = None
        with open('./src/backend_server/backend_server/api/files/file_trees.json', 'r', encoding='utf-8') as file:
            data = file.read()
        return data
    
    @staticmethod
    def post_file():
        pass