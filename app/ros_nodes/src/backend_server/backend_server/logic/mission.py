import time
from backend_server.common import MissionState, MissionStatus
from backend_server.helpers.singleton import Singleton


class MissionData(metaclass=Singleton):
    """
    Singleton to store the data during the mission
    """
    def __init__(self):
        self.start_timestamp: int = 0
        self.stop_timestamp: int = 0
        self.state = MissionState.NOT_STARTED
        self.is_simulation = False  # TODO: run some fort of check (perhaps ROS Channel ID)

    def get_mission_duration(self):
        if self.stop_timestamp != 0:
            return self.stop_timestamp - self.start_timestamp
        else:
            return int(time.time()) - self.start_timestamp

    def terminate_mission(self):
        # TODO: write to the database
        pass

    def get_status(self) -> MissionStatus:
        return MissionStatus(missionState=self.state,
                             elapsedTime=self.get_mission_duration(),
                             startTimestamp=self.start_timestamp,
                             timestamp=int(time.time()),
                             isSimulation=self.is_simulation)
