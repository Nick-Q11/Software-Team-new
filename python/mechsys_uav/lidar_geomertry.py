import numpy as np
import haversine
from marker_detector import MarkerDetection


GRID_SIZE = 8
FOV = 45.0

#höhere Genauigkeit des Winkels, Centers

STEP = 5.625
CENTER = 3.5

def zone_id_to_col_row(zone_id: int, grid_size=GRID_SIZE) -> tuple[int, int]:
    row = zone_id // grid_size
    col = zone_id % grid_size
    return col, row

def zone_to_angles(row: float, col: float) -> tuple[float, float]:
    step = STEP
    center = CENTER
    # col = center - col   VL53L8 horizontal mirror?
    yaw = col * step
    pitch = (center - row) * step
    return yaw, pitch


def angles_to_vector(yaw: float, pitch: float) -> np.ndarray:
    yaw = np.radians(yaw)
    pitch = np.radians(pitch)
    return np.array([
        np.cos(pitch) * np.cos(yaw),
        np.cos(pitch) * np.sin(yaw),
        -np.sin(pitch),
    ])


def rotation_matrix_x(angle: float) -> np.ndarray:
    a = np.radians(angle)
    c, s = np.cos(a), np.sin(a)
    return np.array([[1, 0, 0],
                     [0, c, -s],
                     [0, s, c]])


def rotation_matrix_y(angle: float) -> np.ndarray:
    a = np.radians(angle)
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0, s],
                     [0, 1, 0],
                     [-s, 0, c]])


def rotation_matrix_z(angle: float) -> np.ndarray:
    a = np.radians(angle)
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0],
                     [s, c, 0],
                     [0, 0, 1]])


def gps_offset(lat: float, lon: float, north: float, east: float) -> tuple[float, float]:
    distance = float(np.hypot(north, east))
    if distance == 0:
        return lat, lon

    bearing = float(np.arctan2(east, north))

    return haversine.inverse_haversine(
        (lat, lon),
        distance,
        bearing,
        unit=haversine.Unit.METERS,
    )


def lidar_to_gps(
    drone_lat: float,
    drone_lon: float,
    drone_roll: float,
    drone_pitch: float,
    drone_yaw: float,
    servo_yaw: float,
    servo_pitch: float,
    row: float,
    col: float,
    distance: float,
) -> tuple[float, float]:

    if distance < 0:
        raise ValueError("distance must be non-negative")

    yaw, pitch = zone_to_angles(row, col)


    point = angles_to_vector(yaw, pitch) * distance

    point = (
        rotation_matrix_z(servo_yaw)
        @ rotation_matrix_y(servo_pitch)
        @ point
    )

    point = (
        rotation_matrix_z(drone_yaw)
        @ rotation_matrix_y(drone_pitch)
        @ rotation_matrix_x(drone_roll)
        @ point
    )

    north, east, _ = point

    return gps_offset(drone_lat, drone_lon, north, east)


if __name__ == "__main__":

    # nur zu testen
    lat, lon = lidar_to_gps(
        drone_lat=49.570620,
        drone_lon=11.030250,
        drone_roll=2.5,
        drone_pitch=-1.2,
        drone_yaw=35.0,
        servo_yaw=20.0,
        servo_pitch=-30.0,
        row=5,
        col=2,
        distance=2.45,
    )

    print(f"Marker GPS: {lat:.8f}, {lon:.w8f}")
