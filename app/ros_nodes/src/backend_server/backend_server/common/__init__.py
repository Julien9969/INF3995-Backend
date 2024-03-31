from enum import Enum
from typing import TypedDict


class MissionState(str, Enum):
    ONGOING = "ongoing"
    ENDED = "ended"
    NOT_STARTED = "not-started"


class MissionStatus(TypedDict):
    missionState: MissionState
    elapsedTime: int
    startTimestamp: int
    timestamp: int
    isSimulation: bool


class LogType(str, Enum):
    LOG = "log"
    COMMAND = "command"
    SENSOR = "sensor"


class Log(TypedDict):
    message: str
    timestamp: int
    robotId: int
    eventType: LogType
    missionId: int


class WebsocketsEvents(str, Enum):
    MISSION_START = "mission-start"
    MISSION_END = "mission-end"
    MISSION_STATUS = "mission-status"
    LOG_DATA = "log-data"
    MAP_DATA = "map-data"
    MISSION_MAP = "mission-map"
    ABORT_MISSION = "abort-mission"
    ROBOT_STATUS = "robot-status"
    IDENTIFY_FEEDBACK = "identify-feedback"
    IDENTIFY_REQUEST = "identify-request"
    HEADBACKBASE_FEEDBACK = "headbackbase-feedback"
    HEADBACKBASE_REQUEST = "headbackbase-request"


class Position(TypedDict):
    x: int
    y: int


class RobotInformation(TypedDict):
    id: int
    name: str
    battery: int
    state: str
    lastUpdate: int
    position: Position
    initialPosition: Position


class EmitFeedback(TypedDict):
    timestamp: int
    message: str
    robotId: int
