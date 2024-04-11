import time
import logging

from backend_server.classes.common import RobotInformation, Position, RobotState
from backend_server.classes.singleton import Singleton


class Robot:
    def __init__(self, id: int, initial_position: Position):
        self.id = id
        self.battery = 35
        self.distance = 0.1
        self.initial_position = initial_position
        self.position = initial_position
        self.state = RobotState.IDLE

    def update_position(self, position):
        assert position.x >= 0
        assert position.y >= 0
        self.position = position


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

    def update_battery(self, message: str, robot_id=2):
        battery_level = message.split(":")[1].strip().replace("%", "")
        self.robots[robot_id - 1].battery = battery_level
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
        return self.robots[robot_id]

    def identify_robot(self, robot_id: int):
        for robot in self.robots:
            if robot.id == robot_id:
                robot.state = RobotState.IDENTIFYING
                # TODO: send_cmd_vel(0.1, 0.1, robot_id)

    def head_back_to_base(self):
        for robot in self.robots:
            robot.state = RobotState.HEADING_BACK
            # TODO: head_back_to_base(robot.id)
