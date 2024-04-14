from enum import Enum
from typing import TypedDict


class MissionState(str, Enum):
    ONGOING = "ongoing"
    ENDED = "ended"
    NOT_STARTED = "not-started"


class RobotState(str, Enum):
    IDLE = "idle"
    MOVING = "moving"
    DISCONNECTED = "disconnected"
    IDENTIFYING = "identifying"
    HEADING_BACK_BASE = "heading-back-base"


class MissionStatus(TypedDict):
    missionState: MissionState
    elapsedTime: int
    startTimestamp: int
    robotCount: int
    distance: float
    timestamp: int
    missionId: int
    is_simulation: bool


class LogType(str, Enum):
    LOG = "log"
    COMMAND = "command"
    BATTERY = "battery"
    SENSOR = "sensor"
    COORDS = "coords"


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
    MISSION_MAP = "mission-map"
    ROBOT_STATUS = "robot-status"
    CONNECT = "connect"
    DISCONNECT = "disconnect"
    PING = "ping"
    PONG = "pong"
    HEADBACKBASE = "headbackbase-request"


class Environment(str, Enum):
    SIMULATED = "simulated"
    REAL = "real"


class Position(TypedDict):
    x: int
    y: int


class RobotInformation(TypedDict):
    id: int
    battery: int
    distance: int
    state: RobotState
    lastUpdate: int
    position: Position
    initialPosition: Position


class EmitFeedback(TypedDict):
    timestamp: int
    message: str
    robotId: int


class RobotState(str, Enum):
    IDLE = "idle"
    RUNNING = "running"
    DISCONNECTED = "disconnected"
    IDENTIFYING = "identifying"
    HEADING_BACK = "heading-back"
