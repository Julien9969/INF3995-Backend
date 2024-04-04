from backend_server.helpers.singleton import Singleton
from backend_server.common import RobotInformation, Position, RobotState
from backend_server.db.models import Robot as RobotDB
from backend_server.db.session import SessionLocal


import time


class Robot:
    def __init__(self, id: int, initial_position: Position):
        self.id = id
        self.battery = 100
        self.distance = 0
        self.initial_position = initial_position
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
        robot1 = Robot(1, Position(x=40, y=120))
        robot2 = Robot(2, Position(x=100, y=25))
        self.robots.append(robot1)
        self.robots.append(robot2)

    def connect_robot(self, robot: Robot):
        self.robots.append(robot)

    def disconnect_robot(self, robot: Robot):
        self.robots.remove(robot)

    def get_status(self) -> list[RobotInformation]:
        """
        Generate the object that will be written to the database
        """
        return [RobotInformation(id=robot.id,
                                 name=f"robot{robot.id}",
                                 battery=100,
                                 state=robot.state,
                                 distance=robot.distance,
                                 lastUpdate=int(time.time()),
                                 position=robot.position,
                                 initialPosition=robot.position)
                for robot in self.robots]

    def save_status(self):
        session = SessionLocal()
        for robot in self.robots:
            new_robot_row = RobotDB(id=robot.id,
                                    battery=robot.battery,
                                    distance=robot.distance,
                                    state=robot.state,
                                    last_update=int(time.time()),
                                    position=robot.position,
                                    initial_position=robot.initial_position)
            session.add_all([new_robot_row])
            session.commit()