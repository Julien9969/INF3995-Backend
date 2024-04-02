import logging
import time

from backend_server.common import MissionState, MissionStatus
from backend_server.helpers.singleton import Singleton


class Mission(metaclass=Singleton):
    """
    Singleton to store the data during the mission
    """

    def __init__(self):
        self.start_timestamp: int = 0
        self.stop_timestamp: int = 0
        self.state = MissionState.NOT_STARTED
        self.is_simulation = False  # TODO: run some fort of check (perhaps ROS Channel ID)

    def get_duration(self):
        if self.stop_timestamp != 0:
            return self.stop_timestamp - self.start_timestamp
        else:
            return int(time.time()) - self.start_timestamp

    def start_mission(self):
        if self.state != MissionState.ONGOING:
            self.start_timestamp = int(time.time())
            logging.info("Starting mission node")
            mission = Mission()
            mission.start_mission()
        else:
            logging.info("Mission already started")
        pass

    def stop_mission(self):
        self.stop_timestamp = int(time.time())
        mission = Mission()
        mission.stop_mission()
        self.terminate_mission()
        pass

    def terminate_mission(self):
        # TODO: write to the database
        pass

    def get_status(self) -> MissionStatus:
        return MissionStatus(missionState=self.state,
                             elapsedTime=self.get_duration(),
                             startTimestamp=self.start_timestamp,
                             timestamp=int(time.time()),
                             isSimulation=self.is_simulation)
