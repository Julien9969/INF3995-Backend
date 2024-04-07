
from backend_server.classes.common import LogType, Log
from backend_server.classes.singleton import Singleton


class Logs(metaclass=Singleton):
    def __init__(self):
        self.logs: list[Log] = []

    def add_log(self, log: Log):
        self.logs.append(log)

    def get_logs(self) -> list[Log]:
        return self.logs

    def clear_logs(self):
        self.logs = []
