import asyncio
import numpy as np
from mechsys_uav import UAV
import haversine
import sys
from servo import Servo
from servo import Scanner
from pathlib import Path


BASE_DIR = Path(__file__).resolve().parents[1]
sys.path.append(str(BASE_DIR))

from lidar_interface.wrapper import LidarSensor

# in meters
FLIGHT_ALTITUDE = 2.0
POSITION_TOLERANCE = 0.5

MARKER_DETECTION_THRESHOLD = 0

GRID_SIZE = 4


# Flight Zone Corners
NORTH_EAST_CORNER = (49.57069804930975, 11.030361860205034)
NORTH_WEST_CORNER = (49.57058365455419, 11.030150838986174)
SOUTH_EAST_CORNER = (49.57057898537339, 11.030527456713145)
SOUTH_WEST_CORNER = (49.57046342304837, 11.030320035396016)

FIELD_OF_VIEW = np.radians(22.5)



def get_abs_distance(position1, position2):
    return haversine.haversine(
        position1[:2],
        position2[:2],
        unit=haversine.Unit.METERS,
    )


def interpolate(p1, p2, t):
    return (
        p1[0] + (p2[0] - p1[0]) * t,
        p1[1] + (p2[1] - p1[1]) * t,
    )

def get_pitch_y_matrix(pitch):
    pitch = np.radians(pitch)
    c = np.cos(pitch)
    s = np.sin(pitch)
    return np.array([
        [c, 0, s],
        [0, 1, 0],
        [-s, 0, c]   
    ])
    
def get_yaw_z_matrix(yaw):
    yaw = np.radians(yaw)
    c = np.cos(yaw)
    s = np.sin(yaw)
    return np.array([
        [c, -s, 0],
        [s, c, 0],
        [0, 0, 1],
    ])

def get_distortion_factor_l(pitch, yaw):
    pitch_rad = np.radians(pitch)
    yaw_rad = np.radians(yaw)
    
    numerator_l = np.tan(pitch_rad + FIELD_OF_VIEW) - np.tan(pitch_rad - FIELD_OF_VIEW)
    denominator_l = 2 * np.tan(FIELD_OF_VIEW) * np.cos(yaw_rad)
    factor_l = numerator_l / denominator_l
    
    return factor_l

def get_distortion_factor_w(pitch, yaw):
    pitch_rad = np.radians(pitch)
    yaw_rad = np.radians(yaw)
    
    numerator_w = np.cos(yaw_rad)
    denominator_w = np.cos(pitch_rad + FIELD_OF_VIEW)
    factor_w = numerator_w / denominator_w
    
    return factor_w
    
    
    

def get_position_lidar_from_zone(height_drone, zone, pitch, yaw): 
    
    if not (0 <= zone < 64):
        raise ValueError("Zone must be between 0 and 63 inclusive.")
    
    distance = LidarSensor.get_distance_of_zone(zone)
    
    pitch = np.radians(pitch)
    yaw = np.radians(yaw)
    
    
    
    col = zone % 8
    row = zone // 8

    d = np.sqrt(max(0, distance**2 - height_drone**2))
    
    step = 2 * FIELD_OF_VIEW / 8
    angle_pixel = np.linspace(
    -FIELD_OF_VIEW + step/2,
     FIELD_OF_VIEW - step/2,
     8
    )

    angle_x = angle_pixel[col]
    angle_y = angle_pixel[row]
    
    beam_vector = np.array([
        np.tan(angle_x),
        np.tan(angle_y),
        -1.0
    ])
    
    beam_vector /= np.linalg.norm(beam_vector)

    sensor_system = (get_yaw_z_matrix(yaw) @ get_pitch_y_matrix(pitch)) @ beam_vector

    h = height_drone / (-sensor_system[2])
    x = h * sensor_system[0]
    y = h * sensor_system[1]
    
    pos = x, y

    return pos

def in_coordinates():
    pass

async def get_absolut_lidar_pos(height, uav):
    zones = get_lidar_zone_over_threshold()
    pos_abs_x = []
    pos_abs_y = []
    i = 0
    l = len(zones)
    while i < l:
        pos_rel = get_position_lidar_from_zone(height, zones[i])
        pos_drone = uav.get_position()
        pos_abs_x.append(in_coordinates(pos_rel[0]) + pos_drone[0])
        pos_abs_y.append(in_coordinates(pos_rel[1]) + pos_drone[1])
        i = i + 1
    return pos_abs_x, pos_abs_y
    
async def get_lidar_zone_over_threshold():
    zone = 0;
    relevant_zones = []
    while zone < 64:
        zone, spad = LidarSensor.get_most_spads_i(zone)
        if spad > MARKER_DETECTION_THRESHOLD:
            relevant_zones.append(zone)
        zone = zone+1;
    return relevant_zones

def zone_id_to_angles(zone_id, grid_size=8):
    """Schritt 1: Wandelt die VL53L8CX Zonen-ID (0-63) in Winkel um."""
    row = zone_id // grid_size
    col = zone_id % grid_size
    fov = 45.0
    degrees_per_zone = fov / grid_size
    
    azimuth = (col - (grid_size / 2) + 0.5) * degrees_per_zone
    elevation = ((grid_size / 2) - row - 0.5) * degrees_per_zone
    return azimuth, elevation


def calculate_sensor_vector(azimuth, elevation):
    """Schritt 2: Berechnet den 3D-Richtungsvektor (Feste Höhe 2 Meter)."""
    alpha = np.radians(azimuth)
    beta = np.radians(elevation)
    
    v_sensor = np.array([
        2.0 * np.cos(beta) * np.sin(alpha), # Hier wird 2.0 statt distance genutzt,
        2.0 * np.sin(beta),                 # da die Höhe starr auf 2m bleibt.
        -2.0  
    ])
    return v_sensor


def rotate_vector_imu(v_sensor, roll, pitch, yaw):
    """Schritt 3: Dreht den Vektor anhand der IMU-Fluglage der Drohne."""
    r = np.radians(roll)
    p = np.radians(pitch)
    y = np.radians(yaw)
    
    R_x = np.array([,[0, np.cos(r), -np.sin(r)], [0, np.sin(r), np.cos(r)]])
    R_y = np.array([[np.cos(p), 0, np.sin(p)], , [-np.sin(p), 0, np.cos(p)]])
    R_z = np.array([[np.cos(y), -np.sin(y), 0], [np.sin(y), np.cos(y), 0], ])
    
    R_total = R_z @ R_y @ R_x
    return R_total @ v_sensor


def meter_to_gps_with_haversine(drone_lat, drone_lon, v_world):
    """
    Schritt 4: Nutzt die haversine-Bibliothek für den Positionsversatz.
    """
    dx = v_world  # Meter nach Osten
    dy = v_world  # Meter nach Norden
    
    # 1. Gesamtdistanz am Boden (2D-Hypotenuse) berechnen
    distance_meters = np.sqrt(dx**2 + dy**2)
    
    # 2. Den exakten Kompasswinkel (Bearing) zum Marker berechnen (in Radian)
    # atan2(X, Y) liefert den Winkel relativ zur Nordachse
    bearing_rad = np.arctan2(dx, dy)
    
    # 3. Haversine benötigt den Winkel in Grad/Bogenmaß im Bereich 0 bis 2*Pi
    if bearing_rad < 0:
        bearing_rad += 2 * np.pi
        
    # Startpunkt als Tuple definieren (Breitengrad, Längengrad)
    drone_coords = (drone_lat, drone_lon)
    
    # 4. Berechnung des Zielpunkts via inverse_haversine
    # Wichtig: Die Bibliothek erwartet die Distanz standardmäßig in Kilometern!
    marker_coords = haversine.inverse_haversine(
        point=drone_coords,
        distance=distance_meters / 1000.0,  # Meter in Kilometer umrechnen
        direction=bearing_rad,               # Winkel im Bogenmaß (Radians)
        unit=haversine.Unit.KILOMETERS
    )
    
    return marker_coords[0], marker_coords[1]


# --- HAUPTFUNKTION ---
def get_absolute_marker_position(drone_lat, drone_lon, drone_att, peak_zone_id):
    azimuth, elevation = zone_id_to_angles(peak_zone_id, grid_size=8)
    v_sensor = calculate_sensor_vector(azimuth, elevation)
    v_world = rotate_vector_imu(v_sensor, drone_att, drone_att, drone_att)
    
    # Übergabe an die neue Haversine-Funktion
    marker_lat, marker_lon = meter_to_gps_with_haversine(drone_lat, drone_lon, v_world)
    
    return marker_lat, marker_lon
        