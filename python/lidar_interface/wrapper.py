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

lidar_lib.getZoneStrongestReflectance.argtypes = [ctypes.c_void_p]
lidar_lib.getZoneStrongestReflectance.restype = ctypes.c_int

lidar_lib.getZoneMostSpads.argtypes = [ctypes.c_void_p]
lidar_lib.getZoneMostSpads.restype = ctypes.c_int

lidar_lib.checkMaterial.argtypes = [ctypes.c_void_p, ctypes.c_int]
lidar_lib.checkMaterial.restype = ctypes.c_bool

lidar_lib.printInfoMultiple.argtypes = [ctypes.c_void_p, ctypes.c_int]
lidar_lib.printInfoMultiple.restype = ctypes.c_int

lidar_lib.get_ranging_data.argtypes = [ctypes.c_void_p, ctypes.c_void_p]
lidar_lib.get_ranging_data.restype = ctypes.c_int

lidar_lib.getSpads.argtypes = [ctypes.c_void_p, ctypes.c_int]
lidar_lib.getSpads.restype = ctypes.c_int

lidar_lib.calibrate_glass.argtypes = [ctypes.c_void_p]
lidar_lib.calibrate_glass.restype = ctypes.c_int

lidar_lib.set_sharpener.argtypes = [ctypes.c_void_p, ctypes.c_int]
lidar_lib.set_sharpener.restype = ctypes.c_int

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

    def print_info_matrix(self):
        lidar_lib.printInfoSingle(self._c_ptr)
        
    def print_info_multiple(self, count: int):
        lidar_lib.printInfoMultiple(self._c_ptr, count)
        
    def get_zone_strongest_reflectance(self) -> int:
        return lidar_lib.getZoneStrongestReflectance(self._c_ptr)
    
    def get_zone_most_spads(self) -> int:
        return lidar_lib.getZoneMostSpads(self._c_ptr)
    
    def check_material(self, spad_threshold: int) -> bool:
        return lidar_lib.checkMaterial(self._c_ptr, spad_threshold)
    
    def get_spads_of_zone(self, zone: int) -> int:
        return lidar_lib.getSpads(self._c_ptr, zone)
    
    def calibrate_glass(self, distance_mm: int, reflectance_percent: int) -> int:
        return lidar_lib.calibrate_glass(self._c_ptr, distance_mm, reflectance_percent)
    
    def set_sharpener(self, sharpener_percent: int) -> int:
        return lidar_lib.set_sharpener(self._c_ptr, sharpener_percent)