import logging
import time

from backend_server.common import MissionState, MissionStatus
from backend_server.db.insertions import save_mission, save_logs, save_robots
from backend_server.db.queries import get_new_mission_id
from backend_server.helpers.singleton import Singleton
from backend_server.nodes.clients.mission import MissionNode
from backend_server.logic.logs import Logs
from backend_server.logic.robots import RobotsData


class Mission(metaclass=Singleton):
    """
    Singleton to store the data during the mission
    """

    def __init__(self):
        self.start_timestamp: int = 0
        self.stop_timestamp: int = 0
        self.state = MissionState.NOT_STARTED
        self.mission_id = get_new_mission_id()

    def get_duration(self):
        if self.state == MissionState.NOT_STARTED:
            return 0
        if self.stop_timestamp != 0:  # Mission ended
            return self.stop_timestamp - self.start_timestamp
        else:  # Mission ongoing
            return int(time.time()) - self.start_timestamp

    def start_mission(self):
        if self.state == MissionState.ENDED:  # if restart
            self.stop_timestamp = 0
            logging.info(f"Restarting mission node for mission {self.mission_id}")
        if self.state != MissionState.ONGOING:
            self.start_timestamp = int(time.time())
            self.state = MissionState.ONGOING
            self.mission_id = get_new_mission_id()
            logging.info(f"Starting mission node for mission {self.mission_id}")
            mission = MissionNode()
            mission.start_mission()

    def stop_mission(self):
        self.stop_timestamp = int(time.time())
        self.state = MissionState.ENDED
        mission = MissionNode()
        mission.stop_mission()

        save_mission(self.get_status())

        logs = Logs()
        logs = logs.get_logs()
        save_logs(logs)
        robots = RobotsData()
        robots = robots.save_status()
        save_robots(robots)
        logging.info(f"Stopping mission node for mission {self.mission_id}")

    def get_status(self) -> MissionStatus:
        return MissionStatus(missionId=self.mission_id,
                             missionState=self.state,
                             elapsedTime=self.get_duration(),
                             startTimestamp=self.start_timestamp,
                             timestamp=int(time.time()))
