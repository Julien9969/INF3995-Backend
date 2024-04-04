
from backend_server.common import LogType, Log
from backend_server.db.models import LogDB
from backend_server.db.session import SessionLocal
from backend_server.logic.singleton import Singleton
from backend_server.db.queries import retrieve_logs


class Logs(metaclass=Singleton):
    def __init__(self):
        self.logs = []

    def add_log(self, log: Log):
        self.logs.append(log)

    def get_logs(self):
        return self.logs

    def clear_logs(self):
        self.logs = []

    def save_logs(self):
        for log in self.logs:
            new_log_row = LogDB(mission_id=log['missionId'],
                                robot_id=log['robotId'],
                                log_type=log['eventType'],
                                message=log['message'])
            session = SessionLocal()
            session.add_all([new_log_row])
            session.commit()
        self.clear_logs()

    def retrieve_logs(self):
        logs = retrieve_logs()
        return logs
