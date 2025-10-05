#DroneStates.py
from enum import Enum, auto

class DroneStates(Enum):
    """
    Class for managing drone states
    """
    IDLE = auto()
    ERROR = auto()
    ARMING = auto()
    TAKING_OFF = auto()
    SEARCHING = auto()
    APPROACHING_TARGET = auto()
    HOVERING_OVER_TARGET = auto()
    LANDING = auto()
    EXECUTING_MANEUVER = auto()