import asyncio
import numpy as np
from mechsys_uav import UAV
import haversine
import sys
from pathlib import Path


BASE_DIR = Path(__file__).resolve().parents[1]
sys.path.append(str(BASE_DIR))

from lidar_interface.wrapper import LidarSensor

# in meters
FLIGHT_ALTITUDE = 5.0
POSITION_TOLERANCE = 0.5

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
        [1, 0, 0],
        [0, c, -s],
        [0, s, c],
    ])
    
def get_yaw_z_matrix(yaw):
    yaw = np.radians(yaw)
    c = np.cos(yaw)
    s = np.sin(yaw)
    return np.array([
        [c, 0, s],
        [0, 1, 0],
        [-s, 0, c],
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
    
    col = zone % 8
    row = zone // 8

    d = np.sqrt(max(0, distance**2 - height_drone**2))
    
    angle_pixel = np.linspace(-FIELD_OF_VIEW, FIELD_OF_VIEW, 8)

    angle_x = angle_pixel[col]
    angle_y = angle_pixel[row]
    
    sin_x = np.sin(angle_x)
    sin_y = np.sin(angle_y)

    angle_total = np.sqrt(sin_x**2 + sin_y**2)
    
    if angle_total > 0:
        x = d * sin_x / angle_total
        y = d * sin_y / angle_total
    else:
        x = 0.0
        y = 0.0
    pos = x, y

    return pos
        

    

def generate_lawnmower_waypoints():
    # Generate 16 internal waypoints. All of them lie inside the flight-zone boundary.

    waypoints = []

    for row in range(GRID_SIZE):

        t_row = (row + 1) / (GRID_SIZE + 1)

        left_edge = interpolate(
            NORTH_WEST_CORNER,
            SOUTH_WEST_CORNER,
            t_row,
        )

        right_edge = interpolate(
            NORTH_EAST_CORNER,
            SOUTH_EAST_CORNER,
            t_row,
        )

        row_points = []

        for col in range(GRID_SIZE):

            t_col = (col + 1) / (GRID_SIZE + 1)

            point = interpolate(
                left_edge,
                right_edge,
                t_col,
            )

            row_points.append(point)

        # Lawnmower pattern
        if row % 2 == 1:
            row_points.reverse()

        waypoints.extend(row_points)

    return waypoints


async def fly_to_position(
    uav,
    goal_position,
    relative_altitude=FLIGHT_ALTITUDE,
):

    accepted = await uav.send_goal_position(
        goal_position[0],
        goal_position[1],
        relative_altitude,
    )

    if not accepted:
        return False

    while True:

        await asyncio.sleep(0.1)

        current_position = uav.get_position()

        distance = get_abs_distance(
            current_position,
            goal_position,
        )

        if distance <= POSITION_TOLERANCE:

            return True


async def scan(
    uav,
    scanner,
    waypoint,
):
    """
    Executes a complete servo scan per 180 degrees.

    Returns:
        True：marker found
        False：marker not found
    """
    heading = uav.get_attitude()[2]
 
    # First scan
    await uav.send_goal_position(
        waypoint[0],
        waypoint[1],
        FLIGHT_ALTITUDE,
        heading,
    )

    await scanner.scan()

    # Second scan (180° rotated)
    await uav.send_goal_position(
        waypoint[0],
        waypoint[1],
        FLIGHT_ALTITUDE,
        (heading + 180) % 360,
    )

    await scanner.scan()

    # TODO:
    # Evaluate LiDAR data
    pass

    return False


async def return_home(
    uav,
    home_position,
):

    print("\nReturning home...")

    await fly_to_position(
        uav=uav,
        goal_position=home_position,
    )


async def search(
    uav,
    home_position,
    scanner,
):

    waypoints = generate_lawnmower_waypoints()

    for waypoint in waypoints:

        reached = await fly_to_position(
            uav=uav,
            goal_position=waypoint,
        )

        if not reached:
            continue

        marker_found = await scan(scanner)

        if marker_found:

            await return_home(
                uav,
                home_position,
            )

            return True

    # Search completed without marker detection

    await return_home(
        uav,
        home_position,
    )

    return False