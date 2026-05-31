import ctypes
import os
from pathlib import Path

CUDIR = Path(__file__).resolve().parent
LIBPATH = CUDIR.parents[1] / "lidar" / "core" / "build" / "liblidar.so"

if not LIBPATH.exists():
    raise FileNotFoundError("make ausführen, da .so nicht gefunden")

try:
    lidar_lib = ctypes.CDLL(str(LIBPATH))
except OSError as e:
    raise OSError("Fehler beim laden")

lidar_lib.getSizeOfCalibrateStruct.argtypes = []
lidar_lib.getSizeOfCalibrateStruct.restype = ctypes.c_size_t

lidar_lib.calibrate.argtypes = [ctypes.c_void_p]
lidar_lib.calibrate.restype = ctypes.c_int

lidar_lib.getDistance.argtypes = [ctypes.c_void_p, ctypes.c_int]
lidar_lib.getDistance.restype = ctypes.c_int

lidar_lib.getReflectance.argtypes = [ctypes.c_void_p, ctypes.c_int]
lidar_lib.getReflectance.restype = ctypes.c_int

lidar_lib.getZoneClosestDistance.argtypes = [ctypes.c_void_p]
lidar_lib.getZoneClosestDistance.restype = ctypes.c_int

lidar_lib.printInfoSingle.argtypes = [ctypes.c_void_p]
lidar_lib.printInfoSingle.restype = ctypes.c_int

class LidarSensor:
    def __init__(self):
       
        self.struct_size = lidar_lib.getSizeOfCalibrateStruct()
        
        self._c_buffer = ctypes.create_string_buffer(self.struct_size)
        
        self._c_ptr = ctypes.cast(self._c_buffer, ctypes.c_void_p)
        self.is_calibrated = False

    def init_and_calibrate(self) -> int:
        result = lidar_lib.calibrate(self._c_ptr)
        if result == 0:
            self.is_calibrated = True
        return result

    def get_distance_of_zone(self, zone: int) -> int:
        return lidar_lib.getDistance(self._c_ptr, zone)

    def get_reflectance_of_zone(self, zone: int) -> int:
        return lidar_lib.getReflectance(self._c_ptr, zone)

    def get_closest_zone(self) -> tuple[int, int]:
        zone = lidar_lib.getZoneClosestDistance(self._c_ptr)
        distance = self.get_distance_of_zone(zone)
        return zone, distance

    def print_debug_matrix(self):
        lidar_lib.printInfoSingle(self._c_ptr)