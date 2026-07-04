import sys
from pathlib import Path
from dataclasses import dataclass
BASE_DIR = Path(__file__).resolve().parents[1]
sys.path.append(str(BASE_DIR))

from lidar_interface.wrapper import LidarSensor


SPAD_THRESHOLD = 10000


@dataclass
class MarkerDetection:
    zone: int
    distance: float
    spad: int


class MarkerDetector:

    def __init__(self, lidar: LidarSensor):
        self.lidar = lidar

    def detect(self) -> MarkerDetection | None:

        zone, spad = self.lidar.get_most_spads()

        if spad < SPAD_THRESHOLD:
            return None

        distance = self.lidar.get_distance(zone)

        return MarkerDetection(
            zone=zone,
            distance=distance,
            spad=spad,
        )
