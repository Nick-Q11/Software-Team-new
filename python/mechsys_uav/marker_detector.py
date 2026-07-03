from lidar_interface.wrapper import LidarSensor
from dataclasses import dataclass


SPAD_THRESHOLD = 10000


@dataclass
class MarkerDetection:
    zone: int
    row: float
    col: float
    distance: float
    spad: int


class MarkerDetector:

    def __init__(self, lidar: LidarSensor):
        self.lidar = lidar

    def detect(self) -> MarkerDetection | None:

        zone, spad = self.lidar.get_most_spads()

        if spad < SPAD_THRESHOLD:
            return None

        row = zone // 8
        col = zone % 8

        distance = self.lidar.get_distance(zone)

        return MarkerDetection(
            zone=zone,
            row=row,
            col=col,
            distance=distance,
            spad=spad,
        )
