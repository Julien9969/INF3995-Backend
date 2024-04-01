import time
from backend_server.common import MissionState
from backend_server.helpers.singleton import Singleton


class MissionData(metaclass=Singleton):

    def __init__(self):
        self.start_timestamp: int = 0
        self.stop_timestamp: int = 0
        self.state = MissionState.NOT_STARTED

    def get_mission_state(self):
        return self.state.value

    def get_mission_duration(self):
        if self.stop_timestamp != 0:
            return self.stop_timestamp - self.start_timestamp
        else:
            return int(time.time()) - self.start_timestamp