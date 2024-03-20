from enum import Enum


class Events(Enum):
    MISSION_START = "mission-start"
    MISSION_END = "mission-end"
    MISSION_STATUS = "mission-status"
    MISSION_RESTART = "mission-restart"
    LOG_DATA = "log-data"
