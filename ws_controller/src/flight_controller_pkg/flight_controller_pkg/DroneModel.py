#DroneModel.py
from dataclasses import dataclass
from typing import List

class ParameterValidationError(ValueError):
    """For custom error messagess"""
    pass

@dataclass
class MissionParameters:
    """
    Handles drone parameters for mission
    """
    takeoff_altitude: float
    search_speed: float
    approach_speed: float
    hover_duration: float
    search_plan: List[List[float]]
    def __post_init__(self):
        """Error handle post init"""
        if self.takeoff_altitude <= 0:
            raise ParameterValidationError(f"Height must be greater than zero.\nCurrently set to {self.takeoff_altitude}")
        
        if self.search_speed <= 0 or self.approach_speed <= 0:
            raise ParameterValidationError(f"""Velocity must be greater than zero.\nCurrently search 
            speed set to {self.search_speed} \n approach speed set to {self.approach_speed} """)

        if not self.search_plan:
            raise ParameterValidationError(f"Misson plan can't be empty. Currently {self.search_plan}")
            
        for i, waypoint in enumerate(self.search_plan):
            if len(waypoint) != 3:
                raise ParameterValidationError(f"Waypoint #{i} wrong format. Expected [dx, dy, dz].")