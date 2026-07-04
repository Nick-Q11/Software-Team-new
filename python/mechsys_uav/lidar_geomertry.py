import numpy as np
import haversine
#from marker_detector import MarkerDetection


GRID_SIZE = 8
FOV = 45.0

#höhere Genauigkeit des Winkels, Centers

STEP = 5.625
CENTER = 3.5

def zone_id_to_col_row(zone_id: int, grid_size=GRID_SIZE) -> tuple[int, int]:
    row = zone_id // grid_size
    col = zone_id % grid_size
    #grid um 90 Grad gedreht
    row_n = col
    col_n = grid_size - 1 - row
    return col_n, row_n

def zone_to_angles(row: float, col: float) -> tuple[float, float]:
    step = STEP
    center = CENTER
    yaw = (col - center) * step
    pitch = (center - row) * step
    #more positive pitch --> more northward from center
    #more positive yaw --> more eastward from center
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
    zone_id: int,
    distance: float,
) -> tuple[float, float]:

    if distance < 0:
        raise ValueError("distance must be non-negative")

    col, row = zone_id_to_col_row(zone_id)
    
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
        drone_roll=0,
        drone_pitch=0,
        drone_yaw=0,
        servo_yaw=90,
        servo_pitch=90,
        zone_id=63,
        distance=2,
    )

    print(f"Marker GPS: {lat:.8f}, {lon:.8f}")
