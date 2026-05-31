import asyncio
import numpy as np
from mechsys_uav import UAV
import haversine


# in meters
FLIGHT_ALTITUDE = 5.0
POSITION_TOLERANCE = 0.5

GRID_SIZE = 4


# Flight Zone Corners
NORTH_EAST_CORNER = (49.57069804930975, 11.030361860205034)
NORTH_WEST_CORNER = (49.57058365455419, 11.030150838986174)
SOUTH_EAST_CORNER = (49.57057898537339, 11.030527456713145)
SOUTH_WEST_CORNER = (49.57046342304837, 11.030320035396016)


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


async def scan(scanner):
    """
    Executes a complete servo scan.

    Returns:
        True：marker found
        False：marker not found
    """

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
