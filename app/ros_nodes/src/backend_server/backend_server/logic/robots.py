from backend_server.helpers.singleton import Singleton
from backend_server.common import RobotInformation


class RobotsData(metaclass=Singleton):
    """
    Singleton to store the data during the mission
    """
    def __init__(self):
        self.robots = []

    def connect_robot(self, robot: Robot):
        self.robots.append(robot)

    def disconnect_robot(self, robot: Robot):
        self.robots.remove(robot)

    def get_status(self) -> list[RobotInformation]:
        pass
