import asyncio
import numpy as np
from mechsys_uav import UAV
import haversine
import sys
from servo import Servo, Scanner
from pathlib import Path

BASE_DIR = Path(__file__).resolve().parents[1]
sys.path.append(str(BASE_DIR))

from lidar_interface.wrapper import LidarSensor

# In Metern
FLIGHT_ALTITUDE = 2.0
POSITION_TOLERANCE = 0.5
MARKER_DETECTION_THRESHOLD = 15000
GRID_SIZE = 8  # Der VL53L8CX liefert ein 8x8 Raster für die 64 Zonen

# Flugzonen-Ecken (WGS84)
NORTH_EAST_CORNER = (49.57069804930975, 11.030361860205034)
NORTH_WEST_CORNER = (49.57058365455419, 11.030150838986174)
SOUTH_EAST_CORNER = (49.57057898537339, 11.030527456713145)
SOUTH_WEST_CORNER = (49.57046342304837, 11.030320035396016)

# Volles Sichtfeld des VL53L8CX beträgt 45° horizontal/vertikal
FIELD_OF_VIEW_DEG = 45.0
FIELD_OF_VIEW_8 = 5.625


def get_abs_distance(position1, position2):
    return haversine.haversine(
        position1[:2],
        position2[:2],
        unit=haversine.Unit.METERS,
    )


# --- TRANSFORMATIONSMATRIZEN (SERVO & IMU) ---

def get_pitch_y_matrix(pitch_deg):
    """Drehmatrix für Pitch um die Y-Achse (Nase hoch/runter)."""
    pitch_rad = np.radians(pitch_deg)
    c = np.cos(pitch_rad)
    s = np.sin(pitch_rad)
    return np.array([
        [c, 0, s],
        [0, 1, 0],
        [-s, 0, c]   
    ])
    
def get_yaw_z_matrix(yaw_deg):
    """Drehmatrix für Yaw um die Z-Achse (Kompass-Drehung)."""
    yaw_rad = np.radians(yaw_deg)
    c = np.cos(yaw_rad)
    s = np.sin(yaw_rad)
    return np.array([
        [c, -s, 0],
        [s, c, 0],
        [0, 0, 1]
    ])

def get_roll_x_matrix(roll_deg):
    """Drehmatrix für Roll um die X-Achse (Seitliche Neigung)."""
    roll_rad = np.radians(roll_deg)
    c = np.cos(roll_rad)
    s = np.sin(roll_rad)
    return np.array([
        [1, 0, 0],
        [0, c, -s],
        [0, s, c]
    ])

def get_complete_drone_matrix(roll, pitch, yaw):
    """Kombiniert die IMU-Ausrichtung des Quadcopters (XYZ-Reihenfolge)."""
    return get_yaw_z_matrix(yaw) @ get_pitch_y_matrix(pitch) @ get_roll_x_matrix(roll)


# --- GEOMETRISCHE ANALYSE ---

def zone_id_to_angles(zone_id, grid_size=GRID_SIZE):
    """Wandelt die VL53L8CX Zonen-ID (0-63) in relative Strahlen-Winkel um."""
    row = zone_id // grid_size
    col = zone_id % grid_size
    degrees_per_zone = 5.625
    
    # Winkel relativ zur optischen Mitte des Sensors
    azimuth = (col - 3.5) * degrees_per_zone
    elevation = (3.5 - row) * degrees_per_zone
    return azimuth, elevation


def calculate_sensor_beam(azimuth, elevation):
    """Erzeugt einen richtungsgetreuen Einheitsvektor aus der Sensor-Matrix."""
    alpha = np.radians(azimuth)
    beta = np.radians(elevation)
    
    # Einheitsvektor basierend auf den Tangenten der Sichtwinkel
    beam = np.array([
        np.tan(alpha),
        np.tan(beta),
        -1.0  # Strahl blickt primär nach unten (Z-Achse negativ)
    ])
    return beam / np.linalg.norm(beam)


def meter_to_gps_with_haversine(drone_lat, drone_lon, v_world):
    """Nutzt inverse_haversine, um Bodenmeter in ein GPS-Ziel zu konvertieren."""
    dx = v_world[0]  # Meter nach Osten
    dy = v_world[1]  # Meter nach Norden
    
    distance_meters = np.sqrt(dx**2 + dy**2)
    if distance_meters < 1e-6:
        return drone_lat, drone_lon
        
    bearing_rad = np.arctan2(dx, dy)
    if bearing_rad < 0:
        bearing_rad += 2 * np.pi
        
    drone_coords = (drone_lat, drone_lon)
    marker_coords = haversine.inverse_haversine(
        point=drone_coords,
        distance=distance_meters / 1000.0,
        direction=bearing_rad,
        unit=haversine.Unit.KILOMETERS
    )
    return marker_coords[0], marker_coords[1]


# --- DATEN-AKQUISITION & ASYNC PIPELINE ---

def get_lidar_zones_over_threshold():
    """Liest alle 64 Zonen synchron aus und filtert nach SPAD-Peaks."""
    relevant_zones = []
    for zone in range(64):
        current_zone, spad = LidarSensor.get_most_spads_i(zone)
        if spad > MARKER_DETECTION_THRESHOLD:
            relevant_zones.append(current_zone)
    return relevant_zones


async def get_absolute_lidar_positions(uav: UAV, servo_pitch, servo_yaw):
    """
    Hauptfunktion: Kombiniert Servo-Winkel, Drohnen-Telemetrie und 
    Lidar-Informationen, um den Vektor zweistufig zu rotieren.
    """
    # 1. Lidar-Zonen parallel/asynchron abfragen
    loop = asyncio.get_running_loop()
    zones = await loop.run_in_executor(None, get_lidar_zones_over_threshold)
    
    # 2. Telemetriedaten der Drohne auslesen
    drone_pos = uav.get_position()  # Erwartet (lat, lon, alt)
    drone_att = uav.get_attitude()  # Erwartet (roll, pitch, yaw) in Grad
    
    drone_lat, drone_lon = drone_pos[0], drone_pos[1]
    d_roll, d_pitch, d_yaw = drone_att[0], drone_att[1], drone_att[2]
    
    # 3. Strukturierte Kombinations-Matrizen erstellen
    # Matrix 1: Drehung vom Sensor-Kopf in das Drohnen-Chassis via Servos
    R_gimbal = get_yaw_z_matrix(servo_yaw) @ get_pitch_y_matrix(servo_pitch)
    
    # Matrix 2: Drehung vom Drohnen-Chassis in das globale Erdsystem (IMU)
    R_drone = get_complete_drone_matrix(d_roll, d_pitch, d_yaw)
    
    # Gesamt-Rotations-Kette anwenden: Erst Gimbal-Servos, dann Drohnen-Lage
    R_total = R_drone @ R_gimbal
    
    marker_gps_list = []
    
    for zone in zones:
        # Winkel der detektierten Zone berechnen
        azimuth, elevation = zone_id_to_angles(zone, grid_size=GRID_SIZE)
        
        # Lokalen Richtungsvektor aufstellen
        beam_vector = calculate_sensor_beam(azimuth, elevation)
        
        # Vektor über die gesamte Kette (Lidar -> Gimbal -> Drohne -> Welt) drehen
        rotated_beam = R_total @ beam_vector
        
        # Skalierung auf die exakte Flughöhe von 2.0 Metern am Boden
        # Wenn der gedrehte Strahl den Boden trifft, gilt: h = FLIGHT_ALTITUDE
        h = FLIGHT_ALTITUDE / (-rotated_beam[2])
        v_world = rotated_beam * h
        
        # Versatz in GPS-Koordinaten überführen
        m_lat, m_lon = meter_to_gps_with_haversine(drone_lat, drone_lon, v_world)
        marker_gps_list.append((m_lat, m_lon))
        
    return marker_gps_list
