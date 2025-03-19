import playingwithfusion
from playingwithfusion import TimeOfFlight
import wpilib


class DistanceSensor():
    def __init__(self):
        self.tof = TimeOfFlight(0)
        self.tof.RangingMode(0)

    def get_proximity(self):
        return self.tof.getRange()