import asyncio
import haversine

# in meters
FLIGHT_ALTITUDE = 2.5
POSITION_TOLERANCE = 0.5
WAYPOINT_TIMEOUT = 30.0

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
    """
    Generate 16 internal waypoints
    in serpentine pattern.
    """

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

        # serpentine pattern
        if row % 2 == 1:
            row_points.reverse()

        waypoints.extend(row_points)

    return waypoints


async def fly_to_position(
    uav,
    goal_position,
    relative_altitude=FLIGHT_ALTITUDE,
):
    """
    Fly to GPS position and wait until reached.
    """

    accepted = await uav.send_goal_position(
        goal_position[0],
        goal_position[1],
        relative_altitude,
    )

    if not accepted:
        print("Waypoint rejected.")
        return False

    start_time = asyncio.get_event_loop().time()

    while True:

        await asyncio.sleep(0.1)

        current_position = uav.get_position()

        distance = get_abs_distance(
            current_position,
            goal_position,
        )

        if distance <= POSITION_TOLERANCE:

            print(
                f"Reached waypoint "
                f"(distance={distance:.2f} m)"
            )

            return True

        if (
            asyncio.get_event_loop().time()
            - start_time
            > WAYPOINT_TIMEOUT
        ):

            print(
                f"Waypoint timeout "
                f"(distance={distance:.2f} m)"
            )

            return False


async def scan(
    uav,
    scanner,
    detector,
    waypoint,
):
    """
    Perform two scans:
    - current heading
    - heading + 180°

    Returns:
        measurements
    """

    heading = uav.get_attitude()[2]

    if heading is None:

        print("No heading available.")

        return None

    print(
        f"Scanning at heading "
        f"{heading:.1f}°"
    )

    # First scan
    await uav.send_goal_position(
        waypoint[0],
        waypoint[1],
        FLIGHT_ALTITUDE,
        heading,
    )

    await asyncio.sleep(2)

    measurements_1 = []

    async for measurement in scanner.scan_stream():

        detection = detector.detect()

        measurements_1.append(
            {
                "measurement": measurement,
                "detection": detection,
            }
        )

    await asyncio.sleep(1)

    # Second scan
    second_heading = (
        heading + 180
    ) % 360

    print(
        f"Scanning at heading "
        f"{second_heading:.1f}°"
    )

    await uav.send_goal_position(
        waypoint[0],
        waypoint[1],
        FLIGHT_ALTITUDE,
        second_heading,
    )

    await asyncio.sleep(2)

    measurements_2 = []

    async for measurement in scanner.scan_stream():
    
        detection = detector.detect()
    
        measurements_2.append(
            {
                "measurement": measurement,
                "detection": detection,
            }
        )

    await asyncio.sleep(1)

    measurements = (
        measurements_1
        + measurements_2
    )

    return measurements


async def return_home(
    uav,
    home_position,
):
    """
    Return to home position.
    """

    print("\nReturning home...")

    await fly_to_position(
        uav=uav,
        goal_position=home_position,
    )


async def fly_to_marker(
    uav,
    detection,
    measurement,
    lidar_to_gps,
):
    """
    Compute marker GPS from the detection and fly directly to it.
    """

    position = uav.get_position()
    attitude = uav.get_attitude()

    marker_position = lidar_to_gps(
        drone_lat=position[0],
        drone_lon=position[1],
        drone_roll=attitude[1],
        drone_pitch=attitude[0],
        drone_yaw=attitude[2],
        servo_yaw=90 + measurement["yaw"],
        servo_pitch=90 + measurement["pitch"],
        zone_id=detection.zone,
        distance=detection.distance,
    )

    print(
        f"Marker GPS: "
        f"{marker_position}"
    )

    return await fly_to_position(
        uav=uav,
        goal_position=marker_position,
    )


async def search(
    uav,
    home_position,
    scanner,
):
    """
    Main search routine.
    """

    from marker_detector import MarkerDetector
    from lidar_geometry import lidar_to_gps
    from lidar_interface.wrapper import LidarSensor

    marker_detector = MarkerDetector(LidarSensor())
    waypoints = generate_lawnmower_waypoints()

    print(
        f"Generated "
        f"{len(waypoints)} waypoints."
    )

    for idx, waypoint in enumerate(
        waypoints,
        start=1,
    ):

        print(
            f"\nWaypoint "
            f"{idx}/{len(waypoints)}"
        )

        reached = await fly_to_position(
            uav=uav,
            goal_position=waypoint,
        )

        if not reached:

            print(
                "Skipping waypoint."
            )
            continue

        measurements = await scan(
            uav,
            scanner,
            marker_detector,     
            waypoint,
        )

        if not measurements:
            continue

        marker_found = False

        for result in measurements:

            detection = result["detection"]

            if detection is None:
                continue

            marker_found = True

            print(
                f"\nMarker detected in zone "
                f"{detection.zone}"
            )

            reached_marker = await fly_to_marker(
                uav,
                detection,
                result["measurement"],
                lidar_to_gps,
            )

            await return_home(
                uav,
                home_position,
            )

            return reached_marker

        if not marker_found:
            print("No marker returned by detector.")

    print(
        "\nSearch completed."
    )

    await return_home(
        uav,
        home_position,
    )

    return False
    # Search completes without marker detection
