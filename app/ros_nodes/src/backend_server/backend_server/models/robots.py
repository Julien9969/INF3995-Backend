import time
import logging

from backend_server.classes.common import RobotInformation, Position, RobotState
from backend_server.classes.singleton import Singleton


class Robot:
    def __init__(self, id: int, initial_position: Position):
        self.id = id
        self.battery = None
        self.distance = 0
        self.initial_position = initial_position
        self.position = initial_position
        self.state = RobotState.IDLE

    def update_position(self, position):
        # assert position['x'] >= 0
        # assert position['y'] >= 0
        self.position = position


class RobotsData(metaclass=Singleton):
    """
    Singleton to store the data during the mission
    """

    def __init__(self):
        self.robots: list[Robot] = []

    def reset_robots(self, robot_set:set[int]):
        self.robots = []
        for robot_id in robot_set:
            self.robots.append(Robot(id=robot_id, initial_position=Position(x=0, y=0)))


    def connect_robot(self, robot: Robot):
        self.robots.append(robot)

    def disconnect_robot(self, robot: Robot):
        self.robots.remove(robot)

    def update_battery(self, battery_level:int, robot_id: int):
        for robot in self.robots:
            if robot.id == robot_id:
                robot.battery = battery_level
        logging.debug(f"Robot {robot_id} battery level: {battery_level}")

    def get_robots(self) -> list[RobotInformation]:
        """
        Generate the object that will be written to the database
        """
        return [RobotInformation(id=robot.id,
                                 battery=robot.battery,
                                 state=robot.state,
                                 distance=robot.distance,
                                 lastUpdate=int(time.time()),
                                 position=str(robot.position),
                                 initialPosition=str(robot.position))
                for robot in self.robots]
    
    def get_robot(self, robot_id):
        return self.robots[robot_id-1]

    def identify_robot(self, robot_id: int):
        for robot in self.robots:
            if robot.id == robot_id:
                robot.state = RobotState.IDENTIFYING

    def head_back_to_base(self, robot_id: int=None):
        for robot in self.robots:
            if not robot_id or robot.id == robot_id:
                robot.state = RobotState.HEADING_BACK
                if robot_id:
                    break
