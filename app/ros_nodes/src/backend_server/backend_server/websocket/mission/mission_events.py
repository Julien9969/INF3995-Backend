from enum import Enum


class MissionEvents(Enum):
    MISSION_START = "mission-start"
    MISSION_END = "mission-end"
    MISSION_STATUS = "mission-status"
    TEST_EVENT = "event"