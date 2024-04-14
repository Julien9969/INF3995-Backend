import logging
import time
import subprocess

from backend_server.classes.common import Environment, MissionState, MissionStatus
from backend_server.db.insertions import save_mission
from backend_server.db.queries import get_new_mission_id
from backend_server.classes.singleton import Singleton
from backend_server.ros.clients.mission import MissionNode
from backend_server.models.logs import Logs
from backend_server.models.robots import RobotsData
from backend_server.models.map import MapData

logging.basicConfig(level=logging.INFO)
class Mission(metaclass=Singleton):
    """
    Singleton to store the data during the mission
    """
    mapMergeProcess = None
    def __init__(self):
        self.start_timestamp: int = 0
        self.stop_timestamp: int = 0
        self.state = MissionState.NOT_STARTED
        self.mission_id = get_new_mission_id()
        self.is_simulation: bool = False

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
            answers,environment = mission.start_mission()
            logging.info(environment)

            if(environment == Environment.SIMULATED.value):
                self.is_simulation = True
                if self.mapMergeProcess is None:
                    self.mapMergeProcess = subprocess.Popen(['bash', '/src/app/ros_nodes/start-map-merge-robots.sh'])
            elif(environment == Environment.REAL.value):
                self.is_simulation = False
                if self.mapMergeProcess is None:
                    self.mapMergeProcess = subprocess.Popen(['bash', '/src/app/ros_nodes/start-map-merge-sim.sh'])
            return answers

    def stop_mission(self):
        self.stop_timestamp = int(time.time())
        self.state = MissionState.ENDED
        mission = MissionNode()
        mission.stop_mission()
        logs = Logs()
        robots = RobotsData()
        mission_map = MapData()
        save_mission(self.get_status(),
                     robots.get_robots(),
                     logs.get_logs(),
                     mission_map.get_map())
        
    def head_back_base(self, robot_id:int = None):
        mission = MissionNode()
        if robot_id:
            mission.head_back_base_single(robot_id)
        else:
            mission.head_back_base()

    def check_battery(self,battery_level: int, robot_id: int):
        RobotsData().update_battery(battery_level, robot_id)
        if battery_level < 30:
            self.head_back_base(robot_id)
            RobotsData().head_back_to_base

    def get_status(self) -> MissionStatus:
        return MissionStatus(missionId=self.mission_id,
                             missionState=self.state,
                             elapsedTime=self.get_duration(),
                             startTimestamp=self.start_timestamp,
                             timestamp=int(time.time()),
                             is_simulation=self.is_simulation)
