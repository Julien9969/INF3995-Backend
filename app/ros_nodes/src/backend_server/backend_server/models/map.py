from backend_server.classes.singleton import Singleton


class MapData(metaclass=Singleton):
    def __init__(self):
        self.map = None
        self.map_image = None

    def set_map(self, map):
        self.map = map
        self.map_image = map

    def get_map(self):
        return self.map
