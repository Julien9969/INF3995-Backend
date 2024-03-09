from enum import Enum


class Events(Enum):
    MISSION_START = "mission-start"
    MISSION_END = "mission-end"
    MISSION_STATUS = "mission-status"
    LOG_DATA = "log-data"
