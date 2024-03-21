from enum import Enum


class Events(Enum):
    MISSION_START = "mission-start"
    MISSION_END = "mission-end"
    MISSION_STATUS = "mission-status"
    MISSION_RESTART = "mission-restart"
    MISSION_MAP = "mission-map"
    LOG_DATA = "log-data"
    MAP_DATA = "map-data"
