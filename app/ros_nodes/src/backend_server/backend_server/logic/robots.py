from backend_server.helpers.singleton import Singleton
from backend_server.common import RobotInformation, Position, RobotState


class Robot:
    def __init__(self, id: int, initial_position: Position):
        self.id = id
        self.position = initial_position
        self.state = RobotState.IDLE

    def update_position(self, position):
        assert position.x >= 0
        assert position.y >= 0
        self.position = position

    def poll_for_state(self):
        pass


class RobotsData(metaclass=Singleton):
    """
    Singleton to store the data during the mission
    """
    def __init__(self):
        self.robots: list[Robot] = []

    def connect_robot(self, robot: Robot):
        self.robots.append(robot)

    def disconnect_robot(self, robot: Robot):
        self.robots.remove(robot)

    def get_status(self) -> list[RobotInformation]:
        """
        Generate the object that will be written to the database
        """
        pass
